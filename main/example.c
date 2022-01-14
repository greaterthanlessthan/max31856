#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include <esp_log.h>

#include "max31856.c"

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_CS 5
#define MAX_SPI_HOST VSPI_HOST

#define DRDY_PIN 21

void
app_main (void)
{
  // Bus cfg
  spi_bus_config_t buscfg = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 0,
  };

  // Device cfg
  spi_device_interface_config_t devcfg = max31856_device_interface (PIN_CS);
  static spi_device_handle_t max_spi;

  // Initialize the SPI bus
  esp_err_t ret = spi_bus_initialize (MAX_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK (ret);
  ret = spi_bus_add_device (MAX_SPI_HOST, &devcfg, &max_spi);
  ESP_ERROR_CHECK (ret);

  // install gpio isr service
  ret = gpio_install_isr_service (GPIO_INTR_NEGEDGE);
  ESP_ERROR_CHECK (ret);

  // Begin the temperature reading task
  max31856_start_drdy_pin_task (DRDY_PIN, &max_spi);

  float temperature = 0.0;
  uint8_t fault = 255;
  uint8_t *r;

  // Set open wire detection
  r = max31856_read_register (&max_spi, RW_REG_CR0, 1);
  *r = *r | CR0_OCFAULT0;
  max31856_write_register (&max_spi, RW_REG_CR0, r, 1);

  // Set TC to type K
  r = max31856_read_register (&max_spi, RW_REG_CR1, 1);
  *r = *r | TYPE_K;
  max31856_write_register (&max_spi, RW_REG_CR1, r, 1);

  // automatic conversion is set in DRDY task

  while (true)
    {
      xQueueReceive (MAX31856_TEMP_READ_QUEUE, &temperature, 0);
      xQueueReceive (MAX31856_FAULT_QUEUE, &fault, 0);

      printf ("Temperature: %.4f °C\n", temperature);

      if (fault != 0)
        {
          printf ("Fault! %.02x\n", fault);
        }

      vTaskDelay (pdMS_TO_TICKS (1000));
    }
}
