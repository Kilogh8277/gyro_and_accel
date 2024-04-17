#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"

#define I2C_SDA_GPIO    21      // SDA GPIO
#define I2C_SCL_GPIO    22      // SCL GPIO
#define MPU6050_ADDR    0x68    // Address of the MPU6050 sensor
#define ACCEL_REG       0x3B    // Register for accelerometer data
#define GYRO_REG        0x43    // Register for gyro data

// Define the address of the power management register
#define PWR_MGMT_1_REG      0x6B
#define PWR_MGMT_1_RESET    0x80 // Reset bit
#define PWR_MGMT_1_INITIAL_VALUE 0x01 // Use the best source for the clock

// Setup functions
i2c_master_bus_handle_t setup_i2c_bus(void);
i2c_master_dev_handle_t i2c_add_device(uint8_t dev_addr, i2c_master_bus_handle_t bus_handle);
void i2c_setup_power_management_registers(i2c_master_dev_handle_t dev_handle);

// Data processing functions
void process_mpu6050(i2c_master_dev_handle_t mpu6050_0x68_handle, float* outputData);