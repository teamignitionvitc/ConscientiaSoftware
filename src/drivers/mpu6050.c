/*
                                        ::                                                      
                                        ::                                                      
                                        ::                                                      
                                        ::                                                      
                                        ::                                                      
    ..    ..........    :.      ::      ::     .........  ..    ..........    ...      .        
    ::    ::            : .:.   ::     .::.       ::      ::    ::       :    :: :.    :        
    ::    ::   ..:::    :   .:. ::    ::::::      ::      ::    ::       :    ::   ::  :        
    ::    ::......::    :      :::    ::::::      ::      ::    ::.......:    ::     :::        
                                      ::::::                                                    
                                      :.::.:                                                    
                         .::::          ::          ::::.                                      
                       .::::::::.       ::       .:::::::::                                    
                       ::::::::::::....::::.....:::::::::::                                    
                        .:::::::::::::::::::::::::::::::::.        

                    © Copyright of Ignition Software Department
*/

/**************************************************************************************************
 * File:        mpu6050.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Source file for the MPU6050 6-axis IMU driver.
 *
 * Description:
 *   Implements I2C communication, identity verification, wake-up, and
 *   measurement readout for the InvenSense MPU6050 on the Waveshare
 *   RP2040-Zero using the Pico SDK hardware I2C API.
 *
 *   Operating configuration (written during init):
 *
 *     PWR_MGMT_1 (0x6B) = 0x01
 *       SLEEP    = 0  (wake)
 *       CLKSEL   = 1  (PLL with X-axis gyro reference — recommended)
 *
 *     CONFIG     (0x1A) = 0x03
 *       DLPF_CFG = 3
 *         Accel bandwidth  44 Hz,  delay 4.9 ms
 *         Gyro  bandwidth  42 Hz,  delay 4.8 ms
 *         Internal sample rate = 1 kHz
 *       Balances noise rejection against response time for CanSat
 *       release/descent detection (handled by higher-level modules).
 *
 *     SMPLRT_DIV (0x19) = 0x00
 *       Sample rate = 1000 / (1 + 0) = 1 kHz
 *       Sensor registers always contain a fresh sample when polled
 *       at the 50 Hz main-loop rate (SAMPLE_INTERVAL_IMU_MS = 20 ms).
 *
 *     ACCEL_CONFIG (0x1C) = 0x10
 *       AFS_SEL = 2  -> +/- 8 g   (sensitivity 4096 LSB/g)
 *       Provides adequate headroom for release transients (ejection
 *       shock may reach several g) while retaining useful resolution
 *       for a 100 m drop profile.
 *
 *     GYRO_CONFIG  (0x1B) = 0x08
 *       FS_SEL  = 1  -> +/- 500 deg/s  (sensitivity 65.5 LSB/deg/s)
 *       Sufficient for CanSat tumbling and rotational motion without
 *       saturation, with good resolution.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include "mpu6050.h"
#include "config.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

/**************************************************************************************************
 * Register Addresses (InvenSense MPU-6050 Register Map, rev 4.2)
 **************************************************************************************************/
#define REG_SMPLRT_DIV      (0x19U)
#define REG_CONFIG          (0x1AU)
#define REG_GYRO_CONFIG     (0x1BU)
#define REG_ACCEL_CONFIG    (0x1CU)
#define REG_ACCEL_XOUT_H    (0x3BU)     /* Start of 14-byte burst read            */
#define REG_TEMP_OUT_H      (0x41U)
#define REG_GYRO_XOUT_H     (0x43U)
#define REG_PWR_MGMT_1      (0x6BU)
#define REG_WHO_AM_I        (0x75U)

/** Number of bytes in accel[6] + temp[2] + gyro[6] burst. */
#define REG_BURST_LEN       (14U)

/**************************************************************************************************
 * Configuration Values
 **************************************************************************************************/

/* PWR_MGMT_1: wake from sleep, PLL with X-axis gyro reference */
#define CFG_PWR_MGMT_1      (0x01U)

/* CONFIG: DLPF_CFG = 3  (accel 44 Hz / gyro 42 Hz, 1 kHz internal rate) */
#define CFG_CONFIG           (0x03U)

/* SMPLRT_DIV: sample rate = 1 kHz / (1 + 0) = 1 kHz */
#define CFG_SMPLRT_DIV       (0x00U)

/* ACCEL_CONFIG: AFS_SEL = 2 -> +/- 8 g  (bits [4:3] = 10) */
#define CFG_ACCEL_CONFIG     (0x10U)

/* GYRO_CONFIG:  FS_SEL  = 1 -> +/- 500 deg/s  (bits [4:3] = 01) */
#define CFG_GYRO_CONFIG      (0x08U)

/**************************************************************************************************
 * Sensitivity Scale Factors
 **************************************************************************************************/

/** LSB per g at +/- 8 g range. */
#define ACCEL_SENSITIVITY    (4096.0f)

/** LSB per deg/s at +/- 500 deg/s range. */
#define GYRO_SENSITIVITY     (65.5f)

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/

/** I2C timeout per transaction (us). */
#define I2C_TIMEOUT_US       (10000)

/** Post-reset stabilisation delay (ms). */
#define RESET_DELAY_MS       (50U)

/** Post-wake stabilisation delay (ms). */
#define WAKE_DELAY_MS        (5U)

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool sInitOk = false;

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/
static Mpu6050Result WriteRegister(uint8_t reg, uint8_t value);
static Mpu6050Result ReadRegisters(uint8_t reg, uint8_t *buf, uint8_t len);
static int16_t       AssembleI16(uint8_t msb, uint8_t lsb);

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

Mpu6050Result MPU6050_Init(void)
{
    uint8_t who = 0;
    Mpu6050Result res;

    sInitOk = false;

    /* --- Device reset -------------------------------------------------------- */
    /* Set DEVICE_RESET bit (bit 7) in PWR_MGMT_1 */
    res = WriteRegister(REG_PWR_MGMT_1, 0x80U);
    if (res != MPU6050_OK)
    {
        return res;
    }
    sleep_ms(RESET_DELAY_MS);

    /* --- Wake the device ----------------------------------------------------- */
    /* Clear SLEEP (bit 6), select CLKSEL = 1 (PLL X-gyro) */
    res = WriteRegister(REG_PWR_MGMT_1, CFG_PWR_MGMT_1);
    if (res != MPU6050_OK)
    {
        return res;
    }
    sleep_ms(WAKE_DELAY_MS);

    /* --- Verify identity ----------------------------------------------------- */
    res = ReadRegisters(REG_WHO_AM_I, &who, 1);
    if (res != MPU6050_OK)
    {
        return res;
    }
    if (who != MPU6050_WHO_AM_I_VAL)
    {
        return MPU6050_ERR_WHO_AM_I;
    }

    /* --- Configure sample rate ----------------------------------------------- */
    res = WriteRegister(REG_SMPLRT_DIV, CFG_SMPLRT_DIV);
    if (res != MPU6050_OK)
    {
        return res;
    }

    /* --- Configure DLPF ------------------------------------------------------ */
    res = WriteRegister(REG_CONFIG, CFG_CONFIG);
    if (res != MPU6050_OK)
    {
        return res;
    }

    /* --- Configure accelerometer range --------------------------------------- */
    res = WriteRegister(REG_ACCEL_CONFIG, CFG_ACCEL_CONFIG);
    if (res != MPU6050_OK)
    {
        return res;
    }

    /* --- Configure gyroscope range ------------------------------------------- */
    res = WriteRegister(REG_GYRO_CONFIG, CFG_GYRO_CONFIG);
    if (res != MPU6050_OK)
    {
        return res;
    }

    sInitOk = true;
    return MPU6050_OK;
}

bool MPU6050_IsInitialised(void)
{
    return sInitOk;
}

Mpu6050Result MPU6050_ReadRawAccel(Mpu6050RawAxes *raw)
{
    uint8_t buf[6];
    Mpu6050Result res;

    if (!sInitOk)
    {
        return MPU6050_ERR_NOT_INIT;
    }
    if (raw == NULL)
    {
        return MPU6050_ERR_NOT_INIT;
    }

    res = ReadRegisters(REG_ACCEL_XOUT_H, buf, 6);
    if (res != MPU6050_OK)
    {
        return res;
    }

    raw->x = AssembleI16(buf[0], buf[1]);
    raw->y = AssembleI16(buf[2], buf[3]);
    raw->z = AssembleI16(buf[4], buf[5]);

    return MPU6050_OK;
}

Mpu6050Result MPU6050_ReadRawGyro(Mpu6050RawAxes *raw)
{
    uint8_t buf[6];
    Mpu6050Result res;

    if (!sInitOk)
    {
        return MPU6050_ERR_NOT_INIT;
    }
    if (raw == NULL)
    {
        return MPU6050_ERR_NOT_INIT;
    }

    res = ReadRegisters(REG_GYRO_XOUT_H, buf, 6);
    if (res != MPU6050_OK)
    {
        return res;
    }

    raw->x = AssembleI16(buf[0], buf[1]);
    raw->y = AssembleI16(buf[2], buf[3]);
    raw->z = AssembleI16(buf[4], buf[5]);

    return MPU6050_OK;
} 

Mpu6050Result MPU6050_ReadAccel(Mpu6050AccelData *accel)
{
    Mpu6050RawAxes raw;
    Mpu6050Result res;

    if (accel == NULL)   {
        return MPU6050_ERR_NOT_INIT;
    }

    res = MPU6050_ReadRawAccel(&raw);
    if (res != MPU6050_OK)
    {
        return res;
    }

    accel->x_g = (float)raw.x / ACCEL_SENSITIVITY;
    accel->y_g = (float)raw.y / ACCEL_SENSITIVITY;
    accel->z_g = (float)raw.z / ACCEL_SENSITIVITY;

    return MPU6050_OK;
}

Mpu6050Result MPU6050_ReadGyro(Mpu6050GyroData *gyro)
{
    Mpu6050RawAxes raw;
    Mpu6050Result res;

    if (gyro == NULL)
    {
        return MPU6050_ERR_NOT_INIT;
    }

    res = MPU6050_ReadRawGyro(&raw);
    if (res != MPU6050_OK)
    {
        return res;
    }

    gyro->x_dps = (float)raw.x / GYRO_SENSITIVITY;
    gyro->y_dps = (float)raw.y / GYRO_SENSITIVITY;
    gyro->z_dps = (float)raw.z / GYRO_SENSITIVITY;

    return MPU6050_OK;
}

Mpu6050Result MPU6050_ReadTemperature(float *temperature_c)
{
    uint8_t buf[2];
    int16_t raw;
    Mpu6050Result res;

    if (!sInitOk)
    {
        return MPU6050_ERR_NOT_INIT;
    }
    if (temperature_c == NULL)
    {
        return MPU6050_ERR_NOT_INIT;
    }

    res = ReadRegisters(REG_TEMP_OUT_H, buf, 2);
    if (res != MPU6050_OK)
    {
        return res;
    }

    raw = AssembleI16(buf[0], buf[1]);

    /* Datasheet formula: Temperature (°C) = raw / 340.0 + 36.53 */
    *temperature_c = ((float)raw / 333.87f) + 21.0f;

    return MPU6050_OK;
}

Mpu6050Result MPU6050_Read(Mpu6050Data *data)
{
    uint8_t buf[REG_BURST_LEN];
    int16_t raw;
    Mpu6050Result res;

    if (!sInitOk)
    {
        return MPU6050_ERR_NOT_INIT;
    }
    if (data == NULL)
    {
        return MPU6050_ERR_NOT_INIT;
    }

    /* Burst-read all 14 bytes: accel(6) + temp(2) + gyro(6)
     * starting at ACCEL_XOUT_H (0x3B) through GYRO_ZOUT_L (0x48). */
    res = ReadRegisters(REG_ACCEL_XOUT_H, buf, REG_BURST_LEN);
    if (res != MPU6050_OK)
    {
        return res;
    }

    /* Accelerometer: bytes 0..5 */
    data->accel_x_g = (float)AssembleI16(buf[0],  buf[1])  / ACCEL_SENSITIVITY;
    data->accel_y_g = (float)AssembleI16(buf[2],  buf[3])  / ACCEL_SENSITIVITY;
    data->accel_z_g = (float)AssembleI16(buf[4],  buf[5])  / ACCEL_SENSITIVITY;

    /* Temperature: bytes 6..7 */
    raw = AssembleI16(buf[6], buf[7]);
    data->temperature_c = ((float)raw / 333.87f) + 21.0f;

    /* Gyroscope: bytes 8..13 */
    data->gyro_x_dps = (float)AssembleI16(buf[8],  buf[9])  / GYRO_SENSITIVITY;
    data->gyro_y_dps = (float)AssembleI16(buf[10], buf[11]) / GYRO_SENSITIVITY;
    data->gyro_z_dps = (float)AssembleI16(buf[12], buf[13]) / GYRO_SENSITIVITY;

    return MPU6050_OK;
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

/**
 * \brief   Writes a single byte to an MPU6050 register over I2C.
 */
static Mpu6050Result WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    int ret = i2c_write_timeout_us(I2C0_INSTANCE, MPU6050_I2C_ADDR,
                                   buf, 2, false, I2C_TIMEOUT_US);
    return (ret == 2) ? MPU6050_OK : MPU6050_ERR_I2C;
}

/**
 * \brief   Reads one or more consecutive registers from the MPU6050 over I2C.
 */
static Mpu6050Result ReadRegisters(uint8_t reg, uint8_t *buf, uint8_t len)
{
    int ret;

    /* Send register address (no STOP — repeated start) */
    ret = i2c_write_timeout_us(I2C0_INSTANCE, MPU6050_I2C_ADDR,
                               &reg, 1, true, I2C_TIMEOUT_US);
    if (ret != 1)
    {
        return MPU6050_ERR_I2C;
    }

    /* Read data bytes */
    ret = i2c_read_timeout_us(I2C0_INSTANCE, MPU6050_I2C_ADDR,
                              buf, len, false, I2C_TIMEOUT_US);
    return (ret == len) ? MPU6050_OK : MPU6050_ERR_I2C;
}

/**
 * \brief   Assembles a signed 16-bit value from MSB and LSB bytes.
 */
static int16_t AssembleI16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)((uint16_t)msb << 8 | (uint16_t)lsb);
}
