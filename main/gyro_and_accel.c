#include "i2c_mpu6050.h"

void app_main(void) {
    printf("Initializing I2C comms ...\n");
    // Initialize I2C bus and get device handle
    i2c_master_bus_handle_t master_i2c_handle = setup_i2c_bus();
    if (master_i2c_handle == NULL) {
        printf("Failed to initialize I2C bus\n");
        return;
    }
    else {
        printf("Successfully initialized I2C bus.\n");
    }

    i2c_master_dev_handle_t mpu6050_0x68_handle = i2c_add_device((uint8_t)MPU6050_ADDR, master_i2c_handle);
    if (mpu6050_0x68_handle == NULL) {
        printf("ERROR! I2C bus did not initialize correctly: Address %d\n", MPU6050_ADDR);
        exit(-1);
    }

    printf("Resetting I2C device ...\n");
    // Setup power management registers
    i2c_setup_power_management_registers(mpu6050_0x68_handle);

    // Buffer to store accelerometer and gyro data (6 bytes for accel, 6 bytes for gyro -- X, Y, Z axes)
    float mpu6050_data[12]; // MPU6050 data (accel [0, 1, 2], gyro [3, 4, 5])
    int counter = 0;

    while (true) {
        // Process the acceleration and gyroscopic measurements
        process_mpu6050(mpu6050_0x68_handle, mpu6050_data);

        if (counter % 100 == 0) {
            printf("Accelerometer Data: X=%0.5f, Y=%0.5f, Z=%0.5f\n", mpu6050_data[0], mpu6050_data[1], mpu6050_data[2]);

            printf("Gyro Data: X=%0.5f, Y=%0.5f, Z=%0.5f\n", mpu6050_data[3], mpu6050_data[4], mpu6050_data[5]);
        }

        // Delay before next reading
        vTaskDelay(10 / portTICK_PERIOD_MS);

        counter++;
    }
}
