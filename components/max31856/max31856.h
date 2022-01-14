#include "driver/spi_master.h"
#include "freertos/queue.h"
#include <stdint.h>

#define RW_REG_CR0 0x00    // Configuration 0 register
#define RW_REG_CR1 0x01    // Confgiuration 1 register
#define RW_REG_MASK 0x02   // Fault Mask register
#define RW_REG_CJHF 0x03   // Cold-junction High Fault Threshold
#define RW_REG_CJLF 0x04   // Cold-junction Low Fault Threshold
#define RW_REG_LTHFTH 0x05 // Linearized Temperature High Fault Threshold MSB
#define RW_REG_LTHFTL 0x06 // Linearized Temperature High Fault Threshold LSB
#define RW_REG_LTLFTH 0x07 // Linearized Temperature Low Fault Threshold MSB
#define RW_REG_LTLFTL 0x08 // Linearized Temperature Low Fault Threshold LSB
#define RW_REG_CJTO 0x09   // Cold-Junction Temperature Offset Register
#define RW_REG_CJTH 0x0A   // Cold-Junction Temperature Register, MSB
#define RW_REG_CJTL 0x0B   // Cold-Junction Temperature Register, LSB
#define R_REG_LTCBH 0x0C   // Linearized TC Temperature, Byte 2
#define R_REG_LTCBM 0x0D   // Linearized TC Temperature, Byte 1
#define R_REG_LTCBL 0x0E   // Linearized TC Temperature, Byte 0
#define R_REG_SR 0x0F      // Fault Status Register

#define WRITE_OFFSET 0x80 // Register offset when writing

// CR0 Cfg Bits
#define CR0_CMODE 1 << 7
#define CR0_1SHOT 1 << 6
#define CR0_OCFAULT1 1 << 5
#define CR0_OCFAULT0 1 << 4
#define CR0_CJ 1 << 3
#define CR0_FAULT 1 << 2
#define CR0_FAULTCLR 1 << 1
#define CR0_5060HZ 1 << 0

// CR1 Cfg Bits
#define CR1_AVGSEL2 1 << 6
#define CR1_AVGSEL1 1 << 5
#define CR1_AVGSEL0 1 << 4

// Thermocouple cfg
#define TYPE_B 0b0000
#define TYPE_E 0b0001
#define TYPE_J 0b0010
#define TYPE_K 0b0011 // Default
#define TYPE_N 0b0100
#define TYPE_R 0b0101
#define TYPE_S 0b0110
#define TYPE_T 0b0111

// Fault mask cfg bits
#define FLT_MASK_CJH 1 << 5
#define FLT_MASK_CJL 1 << 4
#define FLT_MASK_TCH 1 << 3
#define FLT_MASK_TCL 1 << 2
#define FLT_MASK_OVUV 1 << 1
#define FL_TMASK_OPN 1 << 0

// Fault bits
#define FAULT_CJRANGE 1 << 7
#define FAULT_TCRANGE 1 << 6
#define FAULT_CJHIGH 1 << 5
#define FAULT_CJLOW 1 << 4
#define FAULT_TCHIGH 1 << 3
#define FAULT_TCLOW 1 << 2
#define FAULT_OVUV 1 << 1
#define FAULT_OPEN 1 << 0

/**
 * @brief when initialized (by calling max31856_start_drdy_pin_task,) this
 * queue whill contain the most recently read temperature from the max31856.
 * This can then be obtained with xQueueReceive
 *
 */
extern xQueueHandle MAX31856_TEMP_READ_QUEUE;

/**
 * @brief writes up to four registers from the max31856, starting with an
 * address given
 *
 * @param spi_handle pointers to the device handle
 * @param address starting address to write to
 * @param data pointer to the array of 8-bit data to write
 * @param num_to_write amount of registers to write to, up to 4. Must be less
 * than or equal to the length of the array at data pointer
 */
void max31856_write_register (spi_device_handle_t *spi_handle, uint8_t address,
                              uint8_t *data, uint8_t num_to_write);

/**
 * @brief reads up to four registers from the max31856, starting with an
 * address given
 *
 * @param spi_handle pointers to the device handle
 * @param address starting address to write to
 * @param num_to_write amount of registers to read, up to 4
 *
 * @return pointer to the 4-element array of bytes return from the transaction
 */
uint8_t *max31856_read_register (spi_device_handle_t *spi_handle,
                                 uint8_t address, uint8_t num_to_read);

/**
 * @brief reads the temperature from the max31856
 *
 * @param spi_handle pointers to the device handle
 *
 * @return temperature reading
 */
float max31856_read_temperature (spi_device_handle_t *spi_handle);

/**
 * @brief Takes a data pin and sets it up as an interrupt to get new
 * temperature when the chip reports that data is ready
 */
void max31856_start_drdy_pin_task (uint32_t drdy_pin,
                                   spi_device_handle_t *spi_handle);

/**
 * @brief returns a device interface for the chip
 *  use spi_bus_add_device (SPI_HOST, &devcfg, &max31856_handle);
 *  To add it to an existing SPI host
 *
 * @return spi_device_interface_config_t
 */
spi_device_interface_config_t max31856_device_interface (uint8_t cs_pin);

/**
 * @brief logs faults from the chip
 *
 */
void max31856_log_faults (uint8_t fault_reg);

/**
 * @brief triggers a conversion and then reads the temperature
 * Conversion takes up to 169ms to complete, so it's recommended to setup a
 * task/queue if calling this often or simply use max31856_start_drdy_pin_task
 * as this function blocks for that time to get the correct reading
 *
 * @param spi_handle pointers to the device handle
 *
 * @return temperature reading
 */
float max31856_oneshot_then_read_temperature (spi_device_handle_t *spi_handle);

/**
 * @brief reads the cold junction (internal chip temperature by default)
 *
 * @param spi_handle pointers to the device handle
 *
 * @return cold junction temperature reading
 */
float max31856_read_cj_temperature (spi_device_handle_t *spi_handle);