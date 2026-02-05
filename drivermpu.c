#include "drivermpu.h"

// Helper function to write a single byte to a register
static void MPU_Write_Register(mss_i2c_instance_t *i2c_instance, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buffer[2];
    write_buffer[0] = reg_addr;
    write_buffer[1] = data;

    // Standard I2C Write: [Address + Write] -> [Reg] -> [Data]
    MSS_I2C_write(i2c_instance, MPU6050_ADDR, write_buffer, 2, MSS_I2C_RELEASE_BUS);
    
    // Blocking wait to ensure configuration is applied before moving on
    MSS_I2C_wait_complete(i2c_instance, MSS_I2C_NO_TIMEOUT);
}

void MPU_Init(mss_i2c_instance_t *i2c_instance) {
    // 1. Reset the device 
    // Write 0x80 to PWR_MGMT_1
    MPU_Write_Register(i2c_instance, MPU_REG_PWR_MGMT_1, MPU_BIT_RESET);
    
    // 2. Wake up the MPU-6050 and set Clock Source
    // By default, MPU-6050 wakes up in Sleep Mode (Bit 6 = 1)
    MPU_Write_Register(i2c_instance, MPU_REG_PWR_MGMT_1, MPU_CLKSEL_PLL_X);

    // 3. Configure Gyroscope Range
    // MPU_REG_GYRO_CONFIG: Default is +/- 250dps (0x00)
    MPU_Write_Register(i2c_instance, MPU_REG_GYRO_CONFIG, 0x00);

    // 4. Configure Accelerometer Range
    // MPU_REG_ACCEL_CONFIG: Default is +/- 2g (0x00)
    MPU_Write_Register(i2c_instance, MPU_REG_ACCEL_CONFIG, 0x00);
}

void MPU_Read_All(mss_i2c_instance_t *i2c_instance, MPU_Data_t *data) {
    uint8_t reg_addr = MPU_REG_ACCEL_XOUT_H; // Start reading at 0x3B
    uint8_t raw_data[14]; // Buffer for 14 bytes (Accel(6) + Temp(2) + Gyro(6))

    /* * Perform an I2C "Write-Read" transaction.
     * 1. Write phase: Send the register address (0x3B).
     * 2. Read phase: Read 14 bytes sequentially. 
     * The MPU auto-increments the register pointer
     */
    MSS_I2C_write_read(
        i2c_instance,           // The I2C instance (I2C0 or I2C1)
        MPU6050_ADDR,           // Target Address (0x68)
        &reg_addr,              // Pointer to the register address to write
        1,                      // Size of offset (1 byte)
        raw_data,               // Buffer to store read data
        14,                     // How many bytes to read
        MSS_I2C_RELEASE_BUS     // Release bus after transaction
    );

    // Wait for the transaction to finish
    if (MSS_I2C_wait_complete(i2c_instance, MSS_I2C_NO_TIMEOUT) == MSS_I2C_SUCCESS) {
        // Combine High and Low bytes into 16-bit signed integers
        // Data is Big-Endian (High byte first)
        
        // Accelerometer
        data->Accel_X = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        data->Accel_Y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        data->Accel_Z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
        
        // Temperature
        data->Temp    = (int16_t)((raw_data[6] << 8) | raw_data[7]);
        
        // Gyroscope
        data->Gyro_X  = (int16_t)((raw_data[8] << 8) | raw_data[9]);
        data->Gyro_Y  = (int16_t)((raw_data[10] << 8) | raw_data[11]);
        data->Gyro_Z  = (int16_t)((raw_data[12] << 8) | raw_data[13]);
    }
}

typedef struct {
    float Ax, Ay, Az;    // Acceleration in g
    float Gx, Gy, Gz;    // Rotation in degrees/sec
    float Temperature;   // Temperature in Celsius
} MPU_Scaled_Data_t;

// Range Definitions (Matches Register Bits for easy usage)
// AFS_SEL (Accel)
#define MPU_RANGE_2G   0
#define MPU_RANGE_4G   1
#define MPU_RANGE_8G   2
#define MPU_RANGE_16G  3

// FS_SEL (Gyro)
#define MPU_RANGE_250DPS  0
#define MPU_RANGE_500DPS  1
#define MPU_RANGE_1000DPS 2
#define MPU_RANGE_2000DPS 3

//Function to convert raw value to scaled physical units based on configured ranges
void MPU_Convert_Data(MPU_Data_t *raw, MPU_Scaled_Data_t *scaled, uint8_t accel_range_param, uint8_t gyro_range_param) {
    float accel_sensitivity = 0.0f;
    float gyro_sensitivity = 0.0f;

    // 1. Determine Accelerometer Sensitivity (LSB/g)
    // Source: Datasheet Section 4.18 (Table)
    switch(accel_range_param) {
        case MPU_RANGE_2G:  accel_sensitivity = 16384.0f; break;
        case MPU_RANGE_4G:  accel_sensitivity = 8192.0f;  break;
        case MPU_RANGE_8G:  accel_sensitivity = 4096.0f;  break;
        case MPU_RANGE_16G: accel_sensitivity = 2048.0f;  break;
        default:            accel_sensitivity = 16384.0f; break; // Default to 2g safety
    }

    // 2. Determine Gyroscope Sensitivity (LSB/dps)
    // Source: Datasheet Section 4.20 (Table)
    switch(gyro_range_param) {
        case MPU_RANGE_250DPS:  gyro_sensitivity = 131.0f;  break;
        case MPU_RANGE_500DPS:  gyro_sensitivity = 65.5f;   break;
        case MPU_RANGE_1000DPS: gyro_sensitivity = 32.8f;   break;
        case MPU_RANGE_2000DPS: gyro_sensitivity = 16.4f;   break;
        default:                gyro_sensitivity = 131.0f;  break; // Default to 250dps safety
    }

    // 3. Perform Conversions
    scaled->Ax = (float)raw->Accel_X / accel_sensitivity;
    scaled->Ay = (float)raw->Accel_Y / accel_sensitivity;
    scaled->Az = (float)raw->Accel_Z / accel_sensitivity;

    scaled->Gx = (float)raw->Gyro_X / gyro_sensitivity;
    scaled->Gy = (float)raw->Gyro_Y / gyro_sensitivity;
    scaled->Gz = (float)raw->Gyro_Z / gyro_sensitivity;

    // 4. Temperature Conversion (Fixed formula)
    // Source: Datasheet Section 4.19
    scaled->Temperature = ((float)raw->Temp / 340.0f) + 36.53f;
}

uint8_t MPU_Who_Am_I(mss_i2c_instance_t *i2c_instance) {
    uint8_t reg_addr = MPU_REG_WHO_AM_I; // 0x75
    uint8_t who_am_i_value = 0;

    /*
     * 1. Write Phase: Send the register address (0x75)
     * 2. Read Phase: Read 1 byte back from the device
     */
    MSS_I2C_write_read(
        i2c_instance,           // I2C instance
        MPU6050_ADDR,           // Device Address (0x68)
        &reg_addr,              // Pointer to register address
        1,                      // Send 1 byte (the register address)
        &who_am_i_value,        // Buffer to store the result
        1,                      // Read 1 byte
        MSS_I2C_RELEASE_BUS     // Release bus after completion
    );

    // Wait for the transaction to complete
    MSS_I2C_wait_complete(i2c_instance, MSS_I2C_NO_TIMEOUT);

    return who_am_i_value;
}

void MPU_Enable_Data_Ready_Interrupt(mss_i2c_instance_t *i2c_instance) {
    uint8_t pin_cfg;
    uint8_t int_enable;

    // 1. Configure the Physical Pin Behavior (Register 0x37 - INT_PIN_CFG)
    // We construct the configuration byte 0xB0 (Binary: 1011 0000)
    // Bit 7 (INT_LEVEL)    = 1 : Active Low (Pin goes to 0V when interrupting)
    // Bit 6 (INT_OPEN)     = 0 : Push-Pull (Drive signal High and Low, no pull-up needed)
    // Bit 5 (LATCH_INT_EN) = 1 : Latch mode (Signal stays Low until you clear it)
    // Bit 4 (INT_RD_CLEAR) = 1 : Clear on Any Read (Reading data registers will clear the interrupt)
    pin_cfg = 0xB0;
    
    MPU_Write_Register(i2c_instance, MPU_REG_INT_PIN_CFG, pin_cfg);

    // 2. Enable the Data Ready Interrupt Source (Register 0x38 - INT_ENABLE)
    // Bit 0 (DATA_RDY_EN)  = 1 : Enable Data Ready interrupt
    int_enable = 0x01;
    
    MPU_Write_Register(i2c_instance, MPU_REG_INT_ENABLE, int_enable);
}

void MPU_Enable_FIFO_Overflow_Interrupt(mss_i2c_instance_t *i2c_instance) {
    uint8_t pin_cfg;
    uint8_t int_enable;
    
    // 1. Configure Physical Pin (Register 0x37 - INT_PIN_CFG)
    // 0xB0 = Active Low, Push-Pull, Latch, Clear on Any Read
    pin_cfg = 0xB0; 
    MPU_Write_Register(i2c_instance, MPU_REG_INT_PIN_CFG, pin_cfg);

    // 2. Enable FIFO Overflow Interrupt (Register 0x38 - INT_ENABLE)
    // Bit 4 is FIFO_OFLOW_EN
    int_enable = 0x10; 
    MPU_Write_Register(i2c_instance, MPU_REG_INT_ENABLE, int_enable);

    // 3. Enable the FIFO Module Operation (Register 0x6A - USER_CTRL)
    // Bit 6 is FIFO_EN. This master switch turns on the FIFO logic.
    // Note: This does NOT put data in yet, it just enables the buffer.
    MPU_Write_Register(i2c_instance, MPU_REG_USER_CTRL, 0x40);

    // 4. Select Data to Stream into FIFO (Register 0x23 - FIFO_EN)
    // We need to put something IN the FIFO for it to eventually overflow.
    // Bit 7: TEMP_FIFO_EN
    // Bit 6: XG_FIFO_EN
    // Bit 5: YG_FIFO_EN
    // Bit 4: ZG_FIFO_EN
    // Bit 3: ACCEL_FIFO_EN
    // Value 0xF8 (Binary 1111 1000) enables ALL sensor data to go to FIFO.
    MPU_Write_Register(i2c_instance, MPU_REG_FIFO_EN, 0xF8);
}

void MPU_Enable_Motion_Interrupt(mss_i2c_instance_t *i2c_instance, uint8_t threshold) {
    uint8_t pin_cfg;
    uint8_t int_enable;

    // 1. Set the Motion Threshold (Register 0x1F - MOT_THR)
    // The device compares the absolute value of the accelerometer reading 
    // against this threshold. 
    MPU_Write_Register(i2c_instance, MPU_REG_MOT_THR, threshold);

    // 2. Set Motion Detection Duration (Register 0x20 - MOT_DUR)
    // Note: Register 0x20 is not explicitly detailed in the provided snippet maps 
    // as "MOT_DUR", but in standard MPU-6050 usage, setting this to 1 
    // ensures the interrupt fires immediately (0ms delay) upon exceeding threshold.
    // If your specific silicon version (Rev C/D) differs, you can skip this, 
    // but standard practice is to write 1 here for instant reaction.
    MPU_Write_Register(i2c_instance, 0x20, 1);

    // 3. Configure Physical Pin (Register 0x37 - INT_PIN_CFG)
    // 0xB0 = Active Low, Push-Pull, Latch until read, Clear on Any Read
    pin_cfg = 0xB0; 
    MPU_Write_Register(i2c_instance, MPU_REG_INT_PIN_CFG, pin_cfg);

    // 4. Enable Motion Detection Interrupt (Register 0x38 - INT_ENABLE)
    // Bit 6 is MOT_EN (0x40)
    int_enable = 0x40;
    MPU_Write_Register(i2c_instance, MPU_REG_INT_ENABLE, int_enable);

}

void MPU_Set_Self_Test(mss_i2c_instance_t *i2c_instance, uint8_t enable) {
    uint8_t gyro_config;
    uint8_t accel_config;

    if (enable) {
        // --- ENABLE SELF TEST ---
        // We must set specific ranges for the test to work:
        // Gyro:  +/- 250 dps (FS_SEL = 0)
        // Accel: +/- 8g      (AFS_SEL = 2, i.e., bit pattern 10)
        
        // Register 0x1B (GYRO_CONFIG)
        // Bits 7,6,5 (XG_ST, YG_ST, ZG_ST) = 1 (Enable Self Test)
        // Bits 4,3   (FS_SEL)              = 00 (+/- 250 dps)
        // Binary: 1110 0000 = 0xE0
        gyro_config = 0xE0;

        // Register 0x1C (ACCEL_CONFIG)
        // Bits 7,6,5 (XA_ST, YA_ST, ZA_ST) = 1 (Enable Self Test)
        // Bits 4,3   (AFS_SEL)             = 10 (+/- 8g)
        // Binary: 1111 0000 = 0xF0
        accel_config = 0xF0;
    } else {
        // --- DISABLE SELF TEST (Return to Defaults) ---
        // You might want to pass your desired default ranges here instead of hardcoding 0.
        
        // Disable Gyro ST, Set range back to default +/- 250dps
        gyro_config = 0x00;
        
        // Disable Accel ST, Set range back to default +/- 2g
        accel_config = 0x00;
    }

    MPU_Write_Register(i2c_instance, MPU_REG_GYRO_CONFIG, gyro_config);
    MPU_Write_Register(i2c_instance, MPU_REG_ACCEL_CONFIG, accel_config);
}

void MPU_Deinit(mss_i2c_instance_t *i2c_instance) {
    // 1. Disable Interrupts immediately
    // Writing 0 to INT_ENABLE stops the MPU from signaling the FPGA.
    MPU_Write_Register(i2c_instance, MPU_REG_INT_ENABLE, 0x00);

    // 2. Reset the Device
    // Register 0x6B (PWR_MGMT_1) Bit 7 is DEVICE_RESET.
    // Writing 0x80 triggers a full reset of internal registers to default.
    MPU_Write_Register(i2c_instance, MPU_REG_PWR_MGMT_1, MPU_BIT_RESET);
}
