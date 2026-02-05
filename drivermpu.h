/*
 * mpu.h
 * * Driver for MPU-6050/6000 using SmartFusion2 MSS I2C
 * Register Map Source: RM-MPU-6000A-00 Rev 4.0
 */

#ifndef MPU_H_
#define MPU_H_

#include "mss_i2c.h"

// I2C Address (AD0 = 0)
// Note: The device address is 7 bits. If AD0 is high, this is 0x69.
#define MPU6050_ADDR            0x68

// ============================================================================
//  REGISTER MAP (Complete List from Datasheet)
// ============================================================================

// Self Test Registers
#define MPU_REG_SELF_TEST_X     0x0D
#define MPU_REG_SELF_TEST_Y     0x0E
#define MPU_REG_SELF_TEST_Z     0x0F
#define MPU_REG_SELF_TEST_A     0x10

// Configuration Registers
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C

// Motion Detection Threshold
#define MPU_REG_MOT_THR         0x1F

// FIFO Enable
#define MPU_REG_FIFO_EN         0x23

// I2C Master Control
#define MPU_REG_I2C_MST_CTRL    0x24

// I2C Slave 0 Control
#define MPU_REG_I2C_SLV0_ADDR   0x25
#define MPU_REG_I2C_SLV0_REG    0x26
#define MPU_REG_I2C_SLV0_CTRL   0x27

// I2C Slave 1 Control
#define MPU_REG_I2C_SLV1_ADDR   0x28
#define MPU_REG_I2C_SLV1_REG    0x29
#define MPU_REG_I2C_SLV1_CTRL   0x2A

// I2C Slave 2 Control
#define MPU_REG_I2C_SLV2_ADDR   0x2B
#define MPU_REG_I2C_SLV2_REG    0x2C
#define MPU_REG_I2C_SLV2_CTRL   0x2D

// I2C Slave 3 Control
#define MPU_REG_I2C_SLV3_ADDR   0x2E
#define MPU_REG_I2C_SLV3_REG    0x2F
#define MPU_REG_I2C_SLV3_CTRL   0x30

// I2C Slave 4 Control
#define MPU_REG_I2C_SLV4_ADDR   0x31
#define MPU_REG_I2C_SLV4_REG    0x32
#define MPU_REG_I2C_SLV4_DO     0x33
#define MPU_REG_I2C_SLV4_CTRL   0x34
#define MPU_REG_I2C_SLV4_DI     0x35

// I2C Master Status
#define MPU_REG_I2C_MST_STATUS  0x36

// Interrupt Configuration
#define MPU_REG_INT_PIN_CFG     0x37
#define MPU_REG_INT_ENABLE      0x38

// Interrupt Status
#define MPU_REG_INT_STATUS      0x3A

// Accelerometer Measurements
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_ACCEL_XOUT_L    0x3C
#define MPU_REG_ACCEL_YOUT_H    0x3D
#define MPU_REG_ACCEL_YOUT_L    0x3E
#define MPU_REG_ACCEL_ZOUT_H    0x3F
#define MPU_REG_ACCEL_ZOUT_L    0x40

// Temperature Measurement
#define MPU_REG_TEMP_OUT_H      0x41
#define MPU_REG_TEMP_OUT_L      0x42

// Gyroscope Measurements
#define MPU_REG_GYRO_XOUT_H     0x43
#define MPU_REG_GYRO_XOUT_L     0x44
#define MPU_REG_GYRO_YOUT_H     0x45
#define MPU_REG_GYRO_YOUT_L     0x46
#define MPU_REG_GYRO_ZOUT_H     0x47
#define MPU_REG_GYRO_ZOUT_L     0x48

// External Sensor Data (24 bytes)
#define MPU_REG_EXT_SENS_DATA_00 0x49
#define MPU_REG_EXT_SENS_DATA_01 0x4A
#define MPU_REG_EXT_SENS_DATA_02 0x4B
#define MPU_REG_EXT_SENS_DATA_03 0x4C
#define MPU_REG_EXT_SENS_DATA_04 0x4D
#define MPU_REG_EXT_SENS_DATA_05 0x4E
#define MPU_REG_EXT_SENS_DATA_06 0x4F
#define MPU_REG_EXT_SENS_DATA_07 0x50
#define MPU_REG_EXT_SENS_DATA_08 0x51
#define MPU_REG_EXT_SENS_DATA_09 0x52
#define MPU_REG_EXT_SENS_DATA_10 0x53
#define MPU_REG_EXT_SENS_DATA_11 0x54
#define MPU_REG_EXT_SENS_DATA_12 0x55
#define MPU_REG_EXT_SENS_DATA_13 0x56
#define MPU_REG_EXT_SENS_DATA_14 0x57
#define MPU_REG_EXT_SENS_DATA_15 0x58
#define MPU_REG_EXT_SENS_DATA_16 0x59
#define MPU_REG_EXT_SENS_DATA_17 0x5A
#define MPU_REG_EXT_SENS_DATA_18 0x5B
#define MPU_REG_EXT_SENS_DATA_19 0x5C
#define MPU_REG_EXT_SENS_DATA_20 0x5D
#define MPU_REG_EXT_SENS_DATA_21 0x5E
#define MPU_REG_EXT_SENS_DATA_22 0x5F
#define MPU_REG_EXT_SENS_DATA_23 0x60

// I2C Slave Data Out
#define MPU_REG_I2C_SLV0_DO     0x63
#define MPU_REG_I2C_SLV1_DO     0x64
#define MPU_REG_I2C_SLV2_DO     0x65
#define MPU_REG_I2C_SLV3_DO     0x66

// I2C Master Delay Control
#define MPU_REG_I2C_MST_DELAY_CTRL 0x67

// Signal Path Reset
#define MPU_REG_SIGNAL_PATH_RESET 0x68

// Motion Detection Control
#define MPU_REG_MOT_DETECT_CTRL 0x69

// User Control
#define MPU_REG_USER_CTRL       0x6A

// Power Management
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_PWR_MGMT_2      0x6C

// FIFO Count
#define MPU_REG_FIFO_COUNT_H    0x72
#define MPU_REG_FIFO_COUNT_L    0x73

// FIFO Read/Write
#define MPU_REG_FIFO_R_W        0x74

// Who Am I
#define MPU_REG_WHO_AM_I        0x75
#define MPU_WHO_AM_I_EXPECTED   0x68

// ============================================================================
//  BIT MASKS AND CONFIGURATION
// ============================================================================

// PWR_MGMT_1 bits
#define MPU_BIT_SLEEP           0x40 //
#define MPU_BIT_RESET           0x80 //
#define MPU_CLKSEL_PLL_X        0x01 //

// USER_CTRL bits [cite: 895]
#define MPU_BIT_I2C_MST_EN      0x20
#define MPU_BIT_FIFO_EN         0x40
#define MPU_BIT_FIFO_RESET      0x04
#define MPU_BIT_I2C_MST_RESET   0x02
#define MPU_BIT_SIG_COND_RESET  0x01

// Structure to hold raw sensor data
typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Temp;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
} MPU_Data_t;

// Checks the device ID. Should return 0x68 if AD0 pin is low.
uint8_t MPU_Who_Am_I(mss_i2c_instance_t *i2c_instance);

// --- Function Prototypes ---

// Initialize the MPU with specific ranges
// accel_range: MPU_RANGE_2G, MPU_RANGE_4G, etc.
// gyro_range:  MPU_RANGE_250DPS, MPU_RANGE_500DPS, etc.
void MPU_Init(mss_i2c_instance_t *i2c_instance, uint8_t accel_range, uint8_t gyro_range);

// Read all raw values (Accel + Temp + Gyro) in one burst
void MPU_Read_All(mss_i2c_instance_t *i2c_instance, MPU_Data_t *data);

// Configures the INT pin to fire an interrupt every time new data is ready.
// Active Low, Push-Pull, Latch until read.
void MPU_Enable_Data_Ready_Interrupt(mss_i2c_instance_t *i2c_instance);

// Configures the INT pin to fire when the internal FIFO buffer is full.
void MPU_Enable_FIFO_Overflow_Interrupt(mss_i2c_instance_t *i2c_instance);

// Configure the MPU to fire an interrupt when motion is detected.
// threshold: 0-255. (1 LSB = 32mg). E.g., 20 = 640mg.
void MPU_Enable_Motion_Interrupt(mss_i2c_instance_t *i2c_instance, uint8_t threshold);

// Enables/Disables the Hardware Self-Test features
// enable: 1 = Enable Self-Test (Applies mechanical force), 0 = Disable (Normal Mode)
void MPU_Set_Self_Test(mss_i2c_instance_t *i2c_instance, uint8_t enable);

// Resets the MPU-6050 and puts it into low-power Sleep Mode.
void MPU_Deinit(mss_i2c_instance_t *i2c_instance);

#endif /* MPU_H_ */
