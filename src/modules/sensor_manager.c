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
 * File:        sensor_manager.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Central sensor management module for CONSCIENTIA.
 *
 * Description:
 *   Initialises I2C0, I2C1, and all sensor drivers.  Runs a non-blocking
 *   update cycle (SensorManager_Update) that samples each sensor at the
 *   rate defined in config.h using elapsed-time comparisons against the
 *   RP2040 millisecond counter (to_ms_since_boot).
 *
 *   Scheduling strategy:
 *     - Each sensor has a static "last sampled" timestamp (ms).
 *     - On every Update() call the elapsed time is checked.
 *     - If elapsed ≥ interval, the sensor is read and the timestamp reset.
 *     - PMS5003 byte-drain poll is called unconditionally every Update()
 *       to prevent PIO RX FIFO overflow regardless of the 1 Hz data interval.
 *     - No blocking waits inside Update().
 *
 *   I2C bus ownership:
 *     I2C0 — BMP280, MPU6050, BH1750 #1, BH1750 #2
 *     I2C1 — BH1750 #3 (address conflict resolution)
 *
 * \todo    Add BMP280 sea-level pressure reference update from GNSS if
 *          barometric altitude is needed for the altitude experiment.
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
#include "sensor_manager.h"
#include "config.h"

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include <string.h>
#include <stddef.h>

/**************************************************************************************************
 * Static BH1750 Instance Descriptors
 *
 * Pre-filled with bus and address; BH1750_Init() completes the rest.
 **************************************************************************************************/
static Bh1750Dev sBh1750_1 = { .i2c = I2C0_INSTANCE, .addr = BH1750_ADDR_LOW  };
static Bh1750Dev sBh1750_2 = { .i2c = I2C0_INSTANCE, .addr = BH1750_ADDR_HIGH };
static Bh1750Dev sBh1750_3 = { .i2c = I2C1_INSTANCE, .addr = BH1750_ADDR_LOW  };

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool   sInitOk  = false;
static SmData sData;

/* Per-sensor last-read timestamp (ms) */
static uint32_t sLastImuMs   = 0;
static uint32_t sLastBaroMs  = 0;
static uint32_t sLastLightMs = 0;
static uint32_t sLastAqMs    = 0;
static uint32_t sLastGnssMs  = 0;
static uint32_t sLastPmMs    = 0;

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/
static void InitI2cBuses(void);
static void PollBaro(uint32_t now_ms);
static void PollImu(uint32_t now_ms);
static void PollLight(uint32_t now_ms);
static void PollPm(uint32_t now_ms);
static void PollAq(uint32_t now_ms);
static void PollGnss(uint32_t now_ms);
static void UpdateHealth(void);

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

SmResult SensorManager_Init(void)
{
    uint8_t fail_count = 0;
    uint8_t total      = 8U;   /* 8 sensor init calls */

    sInitOk = false;
    memset(&sData, 0, sizeof(sData));

    /* --- I2C Buses --- */
    InitI2cBuses();

    /* --- BMP280 --- */
    if (BMP280_Init() != BMP280_OK)
    {
        sData.health.bmp280_ok = false;
        fail_count++;
    }
    else
    {
        sData.health.bmp280_ok = true;
    }

    /* --- MPU6050 --- */
    if (MPU6050_Init() != MPU6050_OK)
    {
        sData.health.mpu6050_ok = false;
        fail_count++;
    }
    else
    {
        sData.health.mpu6050_ok = true;
    }

    /* --- BH1750 #1 (I2C0, 0x23) --- */
    if (BH1750_Init(&sBh1750_1) != BH1750_OK)
    {
        sData.health.bh1750_1_ok = false;
        fail_count++;
    }
    else
    {
        sData.health.bh1750_1_ok = true;
    }

    /* --- BH1750 #2 (I2C0, 0x5C) --- */
    if (BH1750_Init(&sBh1750_2) != BH1750_OK)
    {
        sData.health.bh1750_2_ok = false;
        fail_count++;
    }
    else
    {
        sData.health.bh1750_2_ok = true;
    }

    /* --- BH1750 #3 (I2C1, 0x23) --- */
    if (BH1750_Init(&sBh1750_3) != BH1750_OK)
    {
        sData.health.bh1750_3_ok = false;
        fail_count++;
    }
    else
    {
        sData.health.bh1750_3_ok = true;
    }

    /* --- PMS5003 --- */
    if (PMS5003_Init() != PMS5003_OK)
    {
        sData.health.pms5003_ok = false;
        fail_count++;
    }
    else
    {
        sData.health.pms5003_ok = false;   /* OK once first valid frame arrives */
    }

    /* --- MQ135 --- */
    if (MQ135_Init() != MQ135_OK)
    {
        sData.health.mq135_ok = false;
        fail_count++;
    }
    else
    {
        sData.health.mq135_ok = true;
    }

    /* --- L89HA --- */
    if (L89HA_Init() != L89HA_OK)
    {
        sData.health.l89ha_ok = false;
        fail_count++;
    }
    else
    {
        sData.health.l89ha_ok = false;   /* OK once first valid fix arrives */
    }

    sInitOk = true;

    if (fail_count == 0U)        { return SM_OK; }
    if (fail_count == total)     { return SM_ERR_INIT_ALL; }
    return SM_ERR_INIT_PARTIAL;
}

bool SensorManager_IsInitialised(void)
{
    return sInitOk;
}

SmResult SensorManager_Update(void)
{
    uint32_t now_ms;

    if (!sInitOk)
    {
        return SM_ERR_NOT_INIT;
    }

    now_ms = to_ms_since_boot(get_absolute_time());

    /*
     * PMS5003 byte-drain: called every Update() to prevent PIO RX FIFO
     * overflow (FIFO = 16 bytes deep, frame = 32 bytes @ 9600 baud → ~33 ms
     * per byte; FIFO drains in ~530 ms without polling — well within typical
     * loop rates but call unconditionally for safety).
     */
    (void)PMS5003_Poll();

    /* L89HA NMEA drain over shared I2C0 bus */
    (void)L89HA_Poll();

    PollBaro (now_ms);
    PollImu  (now_ms);
    PollLight(now_ms);
    PollPm   (now_ms);
    PollAq   (now_ms);
    PollGnss (now_ms);

    UpdateHealth();

    return SM_OK;
}

SmResult SensorManager_GetData(SmData *data)
{
    if (!sInitOk)       { return SM_ERR_NOT_INIT; }
    if (data == NULL)   { return SM_ERR_NOT_INIT; }

    *data = sData;
    return SM_OK;
}

SmResult SensorManager_GetHealth(SmHealth *health)
{
    if (!sInitOk)        { return SM_ERR_NOT_INIT; }
    if (health == NULL)  { return SM_ERR_NOT_INIT; }

    *health = sData.health;
    return SM_OK;
}

uint8_t SensorManager_HealthyCount(void)
{
    const SmHealth *h = &sData.health;
    uint8_t count = 0;

    if (h->bmp280_ok)   { count++; }
    if (h->mpu6050_ok)  { count++; }
    if (h->bh1750_1_ok) { count++; }
    if (h->bh1750_2_ok) { count++; }
    if (h->bh1750_3_ok) { count++; }
    if (h->pms5003_ok)  { count++; }
    if (h->mq135_ok)    { count++; }
    if (h->l89ha_ok)    { count++; }

    return count;
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

static void InitI2cBuses(void)
{
    /* I2C0 — BMP280, MPU6050, BH1750 #1 & #2 */
    i2c_init(I2C0_INSTANCE, I2C0_BAUDRATE_HZ);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C0_SDA);
    gpio_pull_up(PIN_I2C0_SCL);

    /* I2C1 — BH1750 #3 */
    i2c_init(I2C1_INSTANCE, I2C1_BAUDRATE_HZ);
    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C1_SDA);
    gpio_pull_up(PIN_I2C1_SCL);
}

static void PollBaro(uint32_t now_ms)
{
    Bmp280Data data;

    if ((now_ms - sLastBaroMs) < SAMPLE_INTERVAL_BARO_MS)
    {
        return;
    }
    sLastBaroMs = now_ms;

    if (!BMP280_IsInitialised())
    {
        sData.baro_valid = false;
        return;
    }

    if (BMP280_Read(&data) == BMP280_OK)
    {
        sData.baro               = data;
        sData.baro_timestamp_ms  = now_ms;
        sData.baro_valid         = true;
        sData.health.bmp280_ok   = true;

        /* Derive altitude from latest pressure */
        (void)BMP280_EstimateAltitude(data.pressure_pa, &sData.baro_altitude_m);
    }
    else
    {
        sData.baro_valid       = false;
        sData.health.bmp280_ok = false;
    }
}

static void PollImu(uint32_t now_ms)
{
    Mpu6050Data data;

    if ((now_ms - sLastImuMs) < SAMPLE_INTERVAL_IMU_MS)
    {
        return;
    }
    sLastImuMs = now_ms;

    if (!MPU6050_IsInitialised())
    {
        sData.imu_valid = false;
        return;
    }

    if (MPU6050_Read(&data) == MPU6050_OK)
    {
        sData.imu              = data;
        sData.imu_timestamp_ms = now_ms;
        sData.imu_valid        = true;
        sData.health.mpu6050_ok = true;
    }
    else
    {
        sData.imu_valid         = false;
        sData.health.mpu6050_ok = false;
    }
}

static void PollLight(uint32_t now_ms)
{
    float lux;

    if ((now_ms - sLastLightMs) < SAMPLE_INTERVAL_LIGHT_MS)
    {
        return;
    }
    sLastLightMs = now_ms;

    /* BH1750 #1 */
    if (BH1750_IsInitialised(&sBh1750_1) &&
        BH1750_ReadLux(&sBh1750_1, &lux) == BH1750_OK)
    {
        sData.light1_lux        = lux;
        sData.light1_valid      = true;
        sData.health.bh1750_1_ok = true;
    }
    else
    {
        sData.light1_valid       = false;
        sData.health.bh1750_1_ok = false;
    }

    /* BH1750 #2 */
    if (BH1750_IsInitialised(&sBh1750_2) &&
        BH1750_ReadLux(&sBh1750_2, &lux) == BH1750_OK)
    {
        sData.light2_lux        = lux;
        sData.light2_valid      = true;
        sData.health.bh1750_2_ok = true;
    }
    else
    {
        sData.light2_valid       = false;
        sData.health.bh1750_2_ok = false;
    }

    /* BH1750 #3 */
    if (BH1750_IsInitialised(&sBh1750_3) &&
        BH1750_ReadLux(&sBh1750_3, &lux) == BH1750_OK)
    {
        sData.light3_lux        = lux;
        sData.light3_valid      = true;
        sData.health.bh1750_3_ok = true;
    }
    else
    {
        sData.light3_valid       = false;
        sData.health.bh1750_3_ok = false;
    }

    sData.light_timestamp_ms = now_ms;
}

static void PollPm(uint32_t now_ms)
{
    Pms5003Data data;

    if ((now_ms - sLastPmMs) < SAMPLE_INTERVAL_PM_MS)
    {
        return;
    }
    sLastPmMs = now_ms;

    /* PMS5003_Poll() already called unconditionally above — just get data */
    if (PMS5003_GetData(&data) == PMS5003_OK)
    {
        sData.pm               = data;
        sData.pm_timestamp_ms  = now_ms;
        sData.pm_valid         = true;
        sData.health.pms5003_ok = true;
    }
    else
    {
        sData.pm_valid = false;
        /* pms5003_ok remains as set by the last successful frame */
    }
}

static void PollAq(uint32_t now_ms)
{
    Mq135Data data;

    if ((now_ms - sLastAqMs) < SAMPLE_INTERVAL_AIR_MS)
    {
        return;
    }
    sLastAqMs = now_ms;

    if (!MQ135_IsInitialised())
    {
        sData.aq_valid = false;
        return;
    }

    if (MQ135_Read(&data) == MQ135_OK)
    {
        sData.aq              = data;
        sData.aq_timestamp_ms = now_ms;
        sData.aq_valid        = true;
        sData.health.mq135_ok  = true;
    }
    else
    {
        sData.aq_valid        = false;
        sData.health.mq135_ok = false;
    }
}

static void PollGnss(uint32_t now_ms)
{
    /* L89HA_Poll() already called above — just snapshot the latest fix */
    if ((now_ms - sLastGnssMs) < SAMPLE_INTERVAL_GNSS_MS)
    {
        return;
    }
    sLastGnssMs = now_ms;

    L89haFix fix;
    if (L89HA_GetFix(&fix) == L89HA_OK)
    {
        sData.gnss              = fix;
        sData.gnss_timestamp_ms = now_ms;
        sData.gnss_valid        = fix.fix_valid;
        sData.health.l89ha_ok   = fix.fix_valid;
    }
    else
    {
        sData.gnss_valid = false;
    }
}

/**
 * \brief   Refreshes aggregate health summary after each update cycle.
 */
static void UpdateHealth(void)
{
    /* Health flags are updated inline in each Poll* function.
     * This function is reserved for future aggregate-level checks
     * (e.g. stale-data watchdog based on timestamp age). */
    (void)0;
}
