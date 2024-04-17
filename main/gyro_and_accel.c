#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#define I2C_SDA_GPIO    21      // SDA GPIO
#define I2C_SCL_GPIO    22      // SCL GPIO
#define MPU6050_ADDR    0x68    // Address of the MPU6050 sensor
#define ACCEL_REG       0x3B    // Register for accelerometer data

// Define the address of the power management register
#define PWR_MGMT_1_REG      0x6B
#define PWR_MGMT_1_RESET    0x80 // Reset bit
#define PWR_MGMT_1_INITIAL_VALUE 0x01 // Use the best source for the clock

i2c_master_dev_handle_t setup_i2c_bus(void) {
    // Initialize I2C bus configuration
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    // Create and add I2C device
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret != ESP_OK) {
        printf("Failed to create I2C bus\n");
        return NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        printf("Failed to add I2C device\n");
        return NULL;
    }

    return dev_handle;
}

void setup_power_management_registers(i2c_master_dev_handle_t dev_handle) {
    // Prepare data to be written to the power management register to perform the reset
    uint8_t data_wr[] = {PWR_MGMT_1_REG, PWR_MGMT_1_RESET};
    size_t data_wr_size = sizeof(data_wr);

    // Write to the power management register to perform the reset
    esp_err_t ret = i2c_master_transmit(dev_handle, data_wr, data_wr_size, 100);
    
    if (ret != ESP_OK) {
        printf("Failed to configure power management register\n");
    }

    // Wait for a brief moment for the reset to complete
    vTaskDelay(pdMS_TO_TICKS(10));

    // Write the initial value to the power management register to set up the sensor
    data_wr[1] = PWR_MGMT_1_INITIAL_VALUE;
    ret = i2c_master_transmit(dev_handle, data_wr, data_wr_size, 100);
    
    if (ret != ESP_OK) {
        printf("Failed to configure power management register\n");
    }

    // Wait for a brief moment for the configuration to take effect
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void app_main(void) {
    // Initialize I2C bus and get device handle
    i2c_master_dev_handle_t master_i2c_handle = setup_i2c_bus();
    if (master_i2c_handle == NULL) {
        printf("Failed to initialize I2C bus\n");
        return;
    }

    // Setup power management registers
    setup_power_management_registers(master_i2c_handle);



    // Buffer to store accelerometer data (6 bytes for X, Y, Z axes)
    uint8_t accel_data[6];

    while (true) {
        // Specify the register address of the accelerometer data
        uint8_t accel_reg = ACCEL_REG;
        esp_err_t ret = i2c_master_transmit(master_i2c_handle, &accel_reg, sizeof(accel_reg), 100);
        if (ret != ESP_OK) {
            printf("Failed to set register address for accelerometer data\n");
            continue; // Skip reading if setting register address fails
        }

        // Read accelerometer data from MPU6050
        ret = i2c_master_receive(master_i2c_handle, accel_data, sizeof(accel_data), 100);
        if (ret == ESP_OK) {
            // Process the accelerometer data
            int16_t accel_x = (accel_data[0] << 8) | accel_data[1];
            int16_t accel_y = (accel_data[2] << 8) | accel_data[3];
            int16_t accel_z = (accel_data[4] << 8) | accel_data[5];

            printf("Accelerometer Data: X=%0.5f, Y=%0.5f, Z=%0.5f\n", (float)(accel_x / 16834.0), (float)(accel_y / 16834.0), (float)(accel_z / 16834.0));
        } else {
            printf("Failed to read accelerometer data\n");
        }

        // Delay before next reading
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
