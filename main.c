#include <stdio.h>
#include "mss_i2c.h"
#include "mpu.h"

// --- Configuration Constants ---
// Define the ranges you want to use here
#define MY_ACCEL_RANGE   MPU_RANGE_2G      // Options: 2G, 4G, 8G, 16G
#define MY_GYRO_RANGE    MPU_RANGE_250DPS  // Options: 250, 500, 1000, 2000

// Global variables for data storage
MPU_Data_t raw_data;          // Holds raw integer values (e.g., 16384)
MPU_Scaled_Data_t sensor_val; // Holds float values (e.g., 1.0 g)

int main() {
    // -------------------------------------------------------------------------
    // 1. Initialize SmartFusion2 I2C Hardware
    // -------------------------------------------------------------------------
    // Address 0x10 is just a dummy address for the SmartFusion2 itself (since it's Master)
    // MSS_I2C_PCLK_DIV_256 sets the bus speed (check your specific PCLK frequency)
    MSS_I2C_init(&g_mss_i2c0, 0x10, MSS_I2C_PCLK_DIV_256);

    // Wake up the sensor and set the ranges defined above
    MPU_Init(&g_mss_i2c0, MY_ACCEL_RANGE, MY_GYRO_RANGE);

    // -------------------------------------------------------------------------
    // 3. Main Data Loop
    // -------------------------------------------------------------------------
    while (1) {
        MPU_Read_All(&g_mss_i2c0, &raw_data);
        MPU_Convert_Data(&raw_data, &sensor_val, MY_ACCEL_RANGE, MY_GYRO_RANGE);

        printf("\nAcceleration: X: %f  Y: %f  Z: %f  ", sensor_val.Ax, sensor_val.Ay, sensor_val.Az);
        printf("\nGryo Value: X: %f  Y: %f  Z: %f   ", sensor_val.Gx, sensor_val.Gy, sensor_val.Gz);
        printf("\nTemperature: %f  ", sensor_val.Temperature);


    }
}
