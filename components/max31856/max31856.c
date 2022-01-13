
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include <esp_log.h>

#include "max31856.h"

#define LOG_MEM_ALLOC_ERROR                                                   \
  ESP_LOGE (TAG, "Could not allocate memory, see %s, line %d", __FILE__,      \
            __LINE__);

const char *TAG = "MAX31856";
static xQueueHandle DRDY_EVT_QUEUE = NULL;
extern xQueueHandle MAX31856_TEMP_READ_QUEUE = NULL;

/**
 * Other functions
 */

/**
 * @brief Converts a 19-bit register into a float
 *  Used for converting temperatue registers into float
 */
static float
max31856_19bit_to_float (uint8_t upper_byte, uint8_t middle_byte,
                         uint8_t lower_byte)
{
  // Last 5 bits are not used, and msb is the sign
  int32_t value = ((upper_byte & 0x7f) << 11) + (middle_byte << 3) + (lower_byte >> 5);
  
  // Handle the sign
  if (upper_byte & 0x80)
    {
      value -= 262144; // 2^18
    }

  // first bit is 1/2^7, but interpreted as 2^0. Scale appropriately.
  float temperature = value / 128.0;
  return temperature;
}

/**
 * SPI functions
 */

spi_device_interface_config_t
max31856_device_interface (uint8_t cs_pin)
{
  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = SPI_MASTER_FREQ_8M,
    .mode = 1,
    .flags = SPI_DEVICE_HALFDUPLEX,
    .spics_io_num = cs_pin,
    .queue_size = 2,

    .dummy_bits = 0,
    .address_bits = 8,
    .command_bits = 0,
  };

  return devcfg;
}

uint8_t *
max31856_read_register (spi_device_handle_t *spi_handle, uint8_t address,
                        uint8_t num_to_read)
{
  if (num_to_read > 4)
    {
      num_to_read = 4;
    }

  // Static as we're returning a pointer
  static spi_transaction_t spi_transaction = {
    .flags = SPI_TRANS_USE_RXDATA,
    .tx_buffer = NULL,
    .rx_buffer = NULL,
  };

  spi_transaction.rxlength = 8 * num_to_read;
  spi_transaction.addr = address & ~WRITE_OFFSET; // ensure not writing

  esp_err_t ret = spi_device_transmit (*spi_handle, &spi_transaction);
  ESP_ERROR_CHECK_WITHOUT_ABORT (ret);

  if (ret == ESP_OK)
    {
      for (uint8_t i = 0; i < num_to_read; i++)
        {
          ESP_LOGD (TAG,
                    "Read value of 0x%02x from MAX31856 at address 0x%02x",
                    spi_transaction.rx_data[i], (address & ~WRITE_OFFSET) + i);
        }
    }

  return spi_transaction.rx_data;
}

void
max31856_write_register (spi_device_handle_t *spi_handle, uint8_t address,
                         uint8_t *data, uint8_t num_to_write)
{
  if (num_to_write > 4)
    {
      num_to_write = 4;
    }

  static spi_transaction_t spi_transaction = {
    .flags = SPI_TRANS_USE_TXDATA,
    .rx_buffer = NULL,
    .tx_buffer = NULL,
    .rxlength = 0,
  };

  spi_transaction.length = 8 * num_to_write;
  spi_transaction.addr = address | WRITE_OFFSET;

  for (uint8_t i = 0; i < num_to_write; i++)
    {
      spi_transaction.tx_data[i] = *(data + i);
      ESP_LOGD (TAG, "Writing value of 0x%02x to address 0x%02x", *(data + i),
                ((address + i) | WRITE_OFFSET));
    }

  esp_err_t ret = spi_device_transmit (*spi_handle, &spi_transaction);
  ESP_ERROR_CHECK_WITHOUT_ABORT (ret);
}

float
max31856_read_temperature (spi_device_handle_t *spi_handle)
{
  // This will be a pointer to an array of 4 bytes, 3 of which have info
  static uint8_t *t;
  t = max31856_read_register (spi_handle, R_REG_LTCBH, 3);
  return max31856_19bit_to_float (*t, *(t + 1), *(t + 2));
}

/**
 * Queue and Task Functions
 */

static void IRAM_ATTR
drdy_isr_handler (void *pin)
{
  uint32_t gpio_num = (uint32_t)pin;
  xQueueSendFromISR (DRDY_EVT_QUEUE, &gpio_num, NULL);
}

static void
read_temperature_task (void *spi_handle)
{
  uint32_t io_num;
  float temperature;
  BaseType_t queue_ret;
  uint8_t no_temp_ct = 0;

  // Queue to store temperature values
  // Size of 1, since most recent temperature best for control
  MAX31856_TEMP_READ_QUEUE = xQueueCreate (1, sizeof (float));
  if (MAX31856_TEMP_READ_QUEUE == NULL)
    {
      LOG_MEM_ALLOC_ERROR
    }

  // Set conversion to automatic
  uint8_t *r = max31856_read_register ((spi_device_handle_t *)spi_handle,
                                       RW_REG_CR0, 1);
  *r = *r | CR0_CMODE;
  max31856_write_register ((spi_device_handle_t *)spi_handle, RW_REG_CR0, r,
                           1);

  for (;;)
    {
      // automatic conversion happens about every 100ms
      queue_ret = xQueueReceive (DRDY_EVT_QUEUE, &io_num, pdMS_TO_TICKS (150));

      // Read the temperature
      temperature
          = max31856_read_temperature ((spi_device_handle_t *)spi_handle);

      // Reset the queue before sending
      // This ensures up to date temperature data is always used
      xQueueReset (MAX31856_TEMP_READ_QUEUE);
      xQueueSend (MAX31856_TEMP_READ_QUEUE, &temperature, portMAX_DELAY);

      ESP_LOGD (TAG, "Sent temperature of %.4f to MAX31856_TEMP_READ_QUEUE",
                temperature);

      // Notify if not getting new value
      no_temp_ct = (queue_ret == pdTRUE) ? 0 : no_temp_ct + 1;

      if (no_temp_ct > 5)
        {
          ESP_LOGE (TAG, "Not getting new temperature value from MAX31856. "
                         "Check CMODE or interrupt set up for pin DRDY.");
        }
    }
}

void
max31856_start_drdy_pin_task (uint32_t drdy_pin,
                              spi_device_handle_t *spi_handle)
{
  esp_err_t ret;
  BaseType_t task_ret;
  uint8_t *r;

  // Configure the pin
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.pin_bit_mask = 1ULL << drdy_pin;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 1;
  ret = gpio_config (&io_conf);
  ESP_ERROR_CHECK_WITHOUT_ABORT (ret);

  // Set interrupt type
  ret = gpio_set_intr_type (drdy_pin, GPIO_INTR_NEGEDGE);
  ESP_ERROR_CHECK_WITHOUT_ABORT (ret);
  // create a queue to handle gpio event from isr
  DRDY_EVT_QUEUE = xQueueCreate (1, sizeof (uint32_t));
  if (DRDY_EVT_QUEUE == NULL)
    {
      LOG_MEM_ALLOC_ERROR
    }

  // start gpio task
  task_ret = xTaskCreate (read_temperature_task, "read_temperature_task", 2048,
                          (void *)spi_handle, 10, NULL);
  if (task_ret == pdPASS)
    {
      ESP_LOGI (TAG,
                "Started temperature reading task; recieve temperatures in "
                "MAX31856_TEMP_READ_QUEUE");
    }
  else
    {
      LOG_MEM_ALLOC_ERROR
    }

  // hook isr handler for specific gpio pin
  ret = gpio_isr_handler_add (drdy_pin, drdy_isr_handler, (void *)drdy_pin);
  ESP_ERROR_CHECK_WITHOUT_ABORT (ret);
  if (ret == ESP_OK)
    {
      ESP_LOGI (TAG, "Attached MAX31856 DRDY pin to ISR on pin %d", drdy_pin);
    }
}
