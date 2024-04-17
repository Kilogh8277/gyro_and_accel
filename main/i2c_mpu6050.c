#include "i2c_mpu6050.h"

i2c_master_bus_handle_t setup_i2c_bus(void) {
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

    return bus_handle;
}

i2c_master_dev_handle_t i2c_add_device(uint8_t dev_addr, i2c_master_bus_handle_t bus_handle) {
    // Define the device configuration
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        printf("Failed to add I2C device\n");
        return NULL;
    }

    return dev_handle;
}

void i2c_setup_power_management_registers(i2c_master_dev_handle_t dev_handle) {
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

void process_mpu6050(i2c_master_dev_handle_t mpu6050_0x68_handle, float* outputData) {
    uint8_t readData[6];    // Buffer for reading data

    // Accelerometer
    uint8_t currReg = ACCEL_REG;

    // Tell the sensor we want the acceleration register when we read
    esp_err_t ret = i2c_master_transmit(mpu6050_0x68_handle, &currReg, sizeof(currReg), 100);
    if (ret != ESP_OK) {
        printf("Failed to set register address for accelerometer data\n");
    }
    else {

        // Read accelerometer data from MPU6050
        ret = i2c_master_receive(mpu6050_0x68_handle, readData, sizeof(readData), 100);
        if (ret == ESP_OK) {
            // Process the accelerometer data
            int16_t accel_x = (readData[0] << 8) | readData[1];
            int16_t accel_y = (readData[2] << 8) | readData[3];
            int16_t accel_z = (readData[4] << 8) | readData[5];

            outputData[0] = accel_x / 16834.0;
            outputData[1] = accel_y / 16834.0;
            outputData[2] = accel_z / 16834.0;
        } else {
            outputData[0] = 0.0;
            outputData[1] = 0.0;
            outputData[2] = 0.0;
            printf("Failed to read accelerometer data\n");
        }
    }

    // Gyroscope
    currReg = GYRO_REG;

    // Tell the sensor we want the gyro register when we read
    ret = i2c_master_transmit(mpu6050_0x68_handle, &currReg, sizeof(currReg), 100);
    if (ret != ESP_OK) {
        printf("Failed to set register address for accelerometer data\n");
    }
    else {

        // Read gyro data from MPU6050
        ret = i2c_master_receive(mpu6050_0x68_handle, readData, 6, 100);
        if (ret == ESP_OK) {
            // Process the gyro data
            int16_t gyroX = (readData[0] << 8) | readData[1];
            int16_t gyroY = (readData[2] << 8) | readData[3];
            int16_t gyroZ = (readData[4] << 8) | readData[5];

            outputData[3] = gyroX / 131.0;
            outputData[4] = gyroY / 131.0;
            outputData[5] = gyroZ / 131.0;
        } else {
            outputData[3] = 0.0;
            outputData[4] = 0.0;
            outputData[5] = 0.0;
            printf("Failed to read gyro data\n");
        }
    }
}