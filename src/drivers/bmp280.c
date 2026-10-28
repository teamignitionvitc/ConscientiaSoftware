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
 * File:        bmp280.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Source file for the BMP280 barometric pressure driver.
 *
 * Description:
 *   Implements I2C communication, calibration readout, and Bosch-datasheet
 *   compensation formulas (32-bit temperature, 64-bit pressure) for the
 *   BMP280 on the Waveshare RP2040-Zero using the Pico SDK hardware I2C API.
 *
 *   Operating configuration (written during init):
 *     ctrl_meas (0xF4) = 0x55
 *       osrs_t  = x2   (bits [7:5] = 010)   — adequate for pressure compensation
 *       osrs_p  = x16  (bits [4:2] = 101)   — high resolution for altitude
 *       mode    = forced (bits [1:0] = 01)   — single-shot, re-triggered per read
 *
 *     config   (0xF5) = 0x08
 *       t_sb    = 0.5 ms  (bits [7:5] = 000) — not used in forced mode
 *       filter  = x4      (bits [4:2] = 010) — moderate IIR smoothing
 *       spi3w_en = 0      (bit  [0]   = 0)   — SPI 4-wire (unused; we use I2C)
 *
 *   Forced mode is chosen so the driver controls exactly when a measurement
 *   occurs.  Each call to BMP280_Read / BMP280_ReadPressure triggers one conversion and waits for it to complete.
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
#include "bmp280.h"
#include "config.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include <math.h>
#include <string.h>

/**************************************************************************************************
 * Register Addresses (Bosch BMP280 datasheet BST-BMP280-DS001)
 **************************************************************************************************/
#define REG_CALIB_START     (0x88U)     /* First calibration register              */
#define REG_CALIB_LEN       (26U)       /* 0x88 .. 0xA1 inclusive = 26 bytes       */
#define REG_CHIP_ID         (0xD0U)
#define REG_RESET           (0xE0U)
#define REG_STATUS          (0xF3U)
#define REG_CTRL_MEAS       (0xF4U)
#define REG_CONFIG          (0xF5U)
#define REG_PRESS_MSB       (0xF7U)     /* Start of 6-byte press+temp burst read  */
#define REG_BURST_LEN       (6U)        /* press[3] + temp[3]                      */

/**************************************************************************************************
 * Configuration Values
 **************************************************************************************************/
/* ctrl_meas = osrs_t(010) | osrs_p(101) | mode(01) = 0b01010101 = 0x55
 * However forced mode requires re-writing mode bits each measurement.
 * We store the "template" with mode = forced (01).
 *
 * osrs_t  = x2  → bits [7:5] = 010
 * osrs_p  = x16 → bits [4:2] = 101
 * mode    = forced → bits [1:0] = 01
 * Combined: 0 1 0 _ 1 0 1 _ 0 1 = 0x55
 */
#define CFG_CTRL_MEAS       (0x55U)

/* config = t_sb(000) | filter(010) | spi3w_en(0)
 * Combined: 0 0 0 _ 0 1 0 _ 0 _ 0 = 0x08
 */
#define CFG_CONFIG          (0x08U)

#define RESET_VALUE         (0xB6U)

/** Maximum time for a single forced measurement to complete (ms).
 *  With osrs_t=x2, osrs_p=x16 the typical measurement time is ~44 ms
 *  (per datasheet Table 14).  Allow generous margin.
 */
#define MEAS_MAX_WAIT_MS    (50U)

/** I2C timeout per transaction (us). */
#define I2C_TIMEOUT_US      (10000)

/**************************************************************************************************
 * Calibration Coefficient Structure (matches BMP280 NVM layout)
 **************************************************************************************************/
typedef struct
{
    uint16_t    dig_T1;
    int16_t     dig_T2;
    int16_t     dig_T3;
    uint16_t    dig_P1;
    int16_t     dig_P2;
    int16_t     dig_P3;
    int16_t     dig_P4;
    int16_t     dig_P5;
    int16_t     dig_P6;
    int16_t     dig_P7;
    int16_t     dig_P8;
    int16_t     dig_P9;
} CalibData;

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static CalibData sCalib;
static int32_t   sTfine;
static float     sSeaLevelPa = BMP280_DEFAULT_SEA_LEVEL_PA;
static bool      sInitOk     = false;

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/

static Bmp280Result WriteRegister(uint8_t reg, uint8_t value);
static Bmp280Result ReadRegisters(uint8_t reg, uint8_t *buf, uint8_t len);
static Bmp280Result ReadCalibration(void);
static Bmp280Result TriggerMeasurement(void);
static Bmp280Result ReadRawBurst(int32_t *raw_temp, int32_t *raw_press);
static int32_t      CompensateTemperature(int32_t adc_T);
static uint32_t     CompensatePressure(int32_t adc_P);

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

Bmp280Result BMP280_Init(void)
{
    uint8_t chip_id = 0;
    Bmp280Result res;

    sInitOk = false;

    /* Soft-reset the device */
    res = WriteRegister(REG_RESET, RESET_VALUE);
    if (res != BMP280_OK)
    {
        return res;
    }

    /* Allow reset to complete (datasheet: startup time 2 ms typical) */
    sleep_ms(5);

    /* Verify chip identity */
    res = ReadRegisters(REG_CHIP_ID, &chip_id, 1);
    if (res != BMP280_OK)
    {
        return res;
    }
    if (chip_id != BMP280_CHIP_ID)
    {
        return BMP280_ERR_CHIP_ID;
    }

    /* Read factory calibration coefficients */
    res = ReadCalibration();
    if (res != BMP280_OK)
    {
        return BMP280_ERR_CALIB;
    }

    /* Write configuration register first (must be written in sleep mode) */
    res = WriteRegister(REG_CONFIG, CFG_CONFIG);
    if (res != BMP280_OK)
    {
        return res;
    }

    /* Write ctrl_meas — this also triggers the first forced measurement */
    res = WriteRegister(REG_CTRL_MEAS, CFG_CTRL_MEAS);
    if (res != BMP280_OK)
    {
        return res;
    }

    sInitOk = true;
    return BMP280_OK;
}

bool BMP280_IsInitialised(void)
{
    return sInitOk;
}

Bmp280Result BMP280_ReadPressure(uint32_t *pressure_pa)
{
    int32_t raw_temp  = 0;
    int32_t raw_press = 0;
    Bmp280Result res;

    if (!sInitOk)
    {
        return BMP280_ERR_NOT_INIT;
    }
    if (pressure_pa == NULL)
    {
        return BMP280_ERR_NOT_INIT;
    }

    res = TriggerMeasurement();
    if (res != BMP280_OK)
    {
        return res;
    }

    res = ReadRawBurst(&raw_temp, &raw_press);
    if (res != BMP280_OK)
    {
        return res;
    }

    /* Temperature must be compensated first to compute t_fine */
    (void)CompensateTemperature(raw_temp);
    *pressure_pa = CompensatePressure(raw_press);
    return BMP280_OK;
}

Bmp280Result BMP280_Read(Bmp280Data *data)
{
    int32_t raw_temp  = 0;
    int32_t raw_press = 0;
    Bmp280Result res;

    if (!sInitOk)
    {
        return BMP280_ERR_NOT_INIT;
    }
    if (data == NULL)
    {
        return BMP280_ERR_NOT_INIT;
    }

    res = TriggerMeasurement();
    if (res != BMP280_OK)
    {
        return res;
    }

    res = ReadRawBurst(&raw_temp, &raw_press);
    if (res != BMP280_OK)
    {
        return res;
    }

    /* BMP280 temperature is used only to calculate t_fine for pressure.
     * Mission temperature comes from the IMU.
     */
    (void)CompensateTemperature(raw_temp);
    data->pressure_pa = CompensatePressure(raw_press);
    return BMP280_OK;
}

void BMP280_SetSeaLevelPressure(float sea_level_pa)
{
    if (sea_level_pa > 0.0f)
    {
        sSeaLevelPa = sea_level_pa;
    }
}

Bmp280Result BMP280_EstimateAltitude(uint32_t pressure_pa, float *altitude_m)
{
    if (!sInitOk)
    {
        return BMP280_ERR_NOT_INIT;
    }
    if (altitude_m == NULL || pressure_pa == 0U)
    {
        return BMP280_ERR_NOT_INIT;
    }

    /*
     * International barometric formula (hypsometric):
     *   altitude = 44330.0 * (1.0 - (P / P0) ^ (1 / 5.255))
     *
     * This is a derived convenience estimate.  Accuracy depends on the
     * reference sea-level pressure and prevailing weather conditions.
     */
    *altitude_m = 44330.0f * (1.0f - powf((float)pressure_pa / sSeaLevelPa,
                                           1.0f / 5.255f));
    return BMP280_OK;
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

/**
 * \brief   Writes a single byte to a BMP280 register over I2C.
 */
static Bmp280Result WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    int ret = i2c_write_timeout_us(I2C0_INSTANCE, BMP280_I2C_ADDR,
                                   buf, 2, false, I2C_TIMEOUT_US);
    return (ret == 2) ? BMP280_OK : BMP280_ERR_I2C;
}

/**
 * \brief   Reads one or more consecutive registers from the BMP280 over I2C.
 */
static Bmp280Result ReadRegisters(uint8_t reg, uint8_t *buf, uint8_t len)
{
    int ret;

    /* Send register address (no STOP — repeated start) */
    ret = i2c_write_timeout_us(I2C0_INSTANCE, BMP280_I2C_ADDR,
                               &reg, 1, true, I2C_TIMEOUT_US);
    if (ret != 1)
    {
        return BMP280_ERR_I2C;
    }

    /* Read data bytes */
    ret = i2c_read_timeout_us(I2C0_INSTANCE, BMP280_I2C_ADDR,
                              buf, len, false, I2C_TIMEOUT_US);
    return (ret == len) ? BMP280_OK : BMP280_ERR_I2C;
}

/**
 * \brief   Reads the 26-byte factory calibration block and unpacks it.
 * \details Registers 0x88–0xA1 are stored in little-endian byte order.
 *          The structure CalibData is laid out identically (RP2040 is LE),
 *          so a direct memcpy is valid.
 */
static Bmp280Result ReadCalibration(void)
{
    uint8_t raw[REG_CALIB_LEN];
    Bmp280Result res;

    res = ReadRegisters(REG_CALIB_START, raw, REG_CALIB_LEN);
    if (res != BMP280_OK)
    {
        return res;
    }

    /* Unpack little-endian pairs explicitly for portability */
    sCalib.dig_T1 = (uint16_t)raw[0]  | ((uint16_t)raw[1]  << 8);
    sCalib.dig_T2 = (int16_t)((uint16_t)raw[2]  | ((uint16_t)raw[3]  << 8));
    sCalib.dig_T3 = (int16_t)((uint16_t)raw[4]  | ((uint16_t)raw[5]  << 8));
    sCalib.dig_P1 = (uint16_t)raw[6]  | ((uint16_t)raw[7]  << 8);
    sCalib.dig_P2 = (int16_t)((uint16_t)raw[8]  | ((uint16_t)raw[9]  << 8));
    sCalib.dig_P3 = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));
    sCalib.dig_P4 = (int16_t)((uint16_t)raw[12] | ((uint16_t)raw[13] << 8));
    sCalib.dig_P5 = (int16_t)((uint16_t)raw[14] | ((uint16_t)raw[15] << 8));
    sCalib.dig_P6 = (int16_t)((uint16_t)raw[16] | ((uint16_t)raw[17] << 8));
    sCalib.dig_P7 = (int16_t)((uint16_t)raw[18] | ((uint16_t)raw[19] << 8));
    sCalib.dig_P8 = (int16_t)((uint16_t)raw[20] | ((uint16_t)raw[21] << 8));
    sCalib.dig_P9 = (int16_t)((uint16_t)raw[22] | ((uint16_t)raw[23] << 8));
    /* Bytes 24-25 (0xA0-0xA1) are reserved / unused */

    return BMP280_OK;
}

/**
 * \brief   Triggers a forced-mode measurement and waits for completion.
 * \details In forced mode the sensor returns to sleep after one conversion.
 *          We re-write ctrl_meas with mode=01 to start a new conversion,
 *          then poll the status register until the measuring bit clears.
 */
static Bmp280Result TriggerMeasurement(void)
{
    Bmp280Result res;
    uint8_t status;
    uint32_t waited = 0;

    /* Write ctrl_meas with forced-mode bits to kick off conversion */
    res = WriteRegister(REG_CTRL_MEAS, CFG_CTRL_MEAS);
    if (res != BMP280_OK)
    {
        return res;
    }

    /* Poll status register until measuring bit (bit 3) clears */
    do
    {
        sleep_ms(1);
        waited++;

        res = ReadRegisters(REG_STATUS, &status, 1);
        if (res != BMP280_OK)
        {
            return res;
        }

        if (waited >= MEAS_MAX_WAIT_MS)
        {
            return BMP280_ERR_I2C;
        }
    } while (status & 0x08U);

    return BMP280_OK;
}

/**
 * \brief   Reads the raw 20-bit temperature and pressure values in one burst.
 * \details Registers 0xF7–0xFC are read as a contiguous 6-byte block:
 *            [press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb]
 */
static Bmp280Result ReadRawBurst(int32_t *raw_temp, int32_t *raw_press)
{
    uint8_t buf[REG_BURST_LEN];
    Bmp280Result res;

    res = ReadRegisters(REG_PRESS_MSB, buf, REG_BURST_LEN);
    if (res != BMP280_OK)
    {
        return res;
    }

    /* Pressure: 20-bit unsigned in buf[0..2] */
    *raw_press = ((int32_t)buf[0] << 12) |
                 ((int32_t)buf[1] << 4)  |
                 ((int32_t)buf[2] >> 4);

    /* Temperature: 20-bit unsigned in buf[3..5] */
    *raw_temp  = ((int32_t)buf[3] << 12) |
                 ((int32_t)buf[4] << 4)  |
                 ((int32_t)buf[5] >> 4);

    return BMP280_OK;
}

/**
 * \brief   Bosch BMP280 temperature compensation (32-bit integer).
 * \details Directly implements the formula from BST-BMP280-DS001 §8.1.
 *          Updates the module-level sTfine which is required by pressure
 *          compensation.
 *
 * \param[in] adc_T  Raw 20-bit temperature value.
 * \return    Compensated temperature in 0.01 °C (e.g. 5123 = 51.23 °C).
 */
static int32_t CompensateTemperature(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = (((adc_T >> 3) - ((int32_t)sCalib.dig_T1 << 1)) *
            ((int32_t)sCalib.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)sCalib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)sCalib.dig_T1))) >> 12) *
            ((int32_t)sCalib.dig_T3)) >> 14;

    sTfine = var1 + var2;
    T = (sTfine * 5 + 128) >> 8;

    return T;
}

/**
 * \brief   Bosch BMP280 pressure compensation (64-bit integer).
 * \details Directly implements the recommended 64-bit formula from
 *          BST-BMP280-DS001 §8.1.  Returns pressure in Pa (integer).
 *          sTfine must be computed first via CompensateTemperature().
 *
 *          The intermediate Q24.8 result is divided by 256 to yield Pa.
 *
 * \param[in] adc_P  Raw 20-bit pressure value.
 * \return    Compensated pressure in Pa (e.g. 96386 = 963.86 hPa).
 *            Returns 0 if a division-by-zero would occur.
 */
static uint32_t CompensatePressure(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)sTfine) - 128000;
    var2 = var1 * var1 * (int64_t)sCalib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)sCalib.dig_P5) << 17);
    var2 = var2 + (((int64_t)sCalib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)sCalib.dig_P3) >> 8) +
           ((var1 * (int64_t)sCalib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)sCalib.dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0U;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)sCalib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)sCalib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)sCalib.dig_P7) << 4);

    /* p is now in Q24.8 format — divide by 256 to get Pa */
    return (uint32_t)(p >> 8);
}
