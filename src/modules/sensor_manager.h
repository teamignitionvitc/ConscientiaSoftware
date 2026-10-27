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
 * File:        sensor_manager.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Central sensor management module header.
 * \details     Aggregates all CONSCIENTIA flight sensors (BMP280, MPU6050,
 *              BH1750 ×3, PMS5003, MQ135, L89HA) into a single update cycle,
 *              scheduling each at the rate defined in config.h and maintaining
 *              a central latest-data snapshot with timestamps and health flags.
 *
 *              This module does NOT perform payload processing, flight-state
 *              decisions, recovery logic, SD logging, LoRa or telemetry.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef SENSOR_MANAGER_H_
#define SENSOR_MANAGER_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "drivers/bmp280.h"
#include "drivers/mpu6050.h"
#include "drivers/bh1750.h"
#include "drivers/pms5003.h"
#include "drivers/mq135.h"
#include "drivers/l89ha.h"

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by sensor manager functions.
 */
typedef enum
{
	SM_OK = 0,              /**< Operation succeeded.                         */
	SM_ERR_INIT_PARTIAL,    /**< One or more sensors failed to initialise.    */
	SM_ERR_INIT_ALL,        /**< Every sensor failed to initialise.           */
	SM_ERR_NOT_INIT         /**< Module used before SensorManager_Init().     */
} SmResult;

/**
 * \brief   Per-sensor health / status flags.
 */
typedef struct
{
	bool bmp280_ok;         /**< BMP280 initialised and last read succeeded.  */
	bool mpu6050_ok;        /**< MPU6050 initialised and last read succeeded. */
	bool bh1750_1_ok;       /**< BH1750 #1 (I2C0, 0x23) last read OK.        */
	bool bh1750_2_ok;       /**< BH1750 #2 (I2C0, 0x5C) last read OK.        */
	bool bh1750_3_ok;       /**< BH1750 #3 (I2C1, 0x23) last read OK.        */
	bool pms5003_ok;        /**< PMS5003 has produced at least one valid frame.*/
	bool mq135_ok;          /**< MQ135 last read succeeded.                   */
	bool l89ha_ok;          /**< L89HA has at least one valid fix.            */
} SmHealth;

/**
 * \brief   Latest readings from all flight sensors with timestamps.
 */
typedef struct
{
	/* --- BMP280 Barometric Pressure / Temperature --- */
	Bmp280Data  baro;                   /**< Compensated pressure + temperature. */
	float       baro_altitude_m;        /**< Barometric altitude estimate (m).   */
	uint32_t    baro_timestamp_ms;      /**< Time of last valid baro read (ms).  */
	bool        baro_valid;

	/* --- MPU6050 6-Axis IMU --- */
	Mpu6050Data imu;                    /**< Accel (g), gyro (dps), die temp.   */
	uint32_t    imu_timestamp_ms;
	bool        imu_valid;

	/* --- BH1750 Ambient Light ×3 --- */
	float       light1_lux;             /**< BH1750 #1 illuminance (I2C0, 0x23).*/
	float       light2_lux;             /**< BH1750 #2 illuminance (I2C0, 0x5C).*/
	float       light3_lux;             /**< BH1750 #3 illuminance (I2C1, 0x23).*/
	uint32_t    light_timestamp_ms;
	bool        light1_valid;
	bool        light2_valid;
	bool        light3_valid;

	/* --- PMS5003 Particulate Matter --- */
	Pms5003Data pm;                     /**< PM1.0/2.5/10 + particle counts.    */
	uint32_t    pm_timestamp_ms;
	bool        pm_valid;

	/* --- MQ135 Air Quality --- */
	Mq135Data   aq;                     /**< Raw ADC, ADC voltage, sensor V.    */
	uint32_t    aq_timestamp_ms;
	bool        aq_valid;

	/* --- L89HA GNSS --- */
	L89haFix    gnss;                   /**< Lat, lon, alt, speed, course, …   */
	uint32_t    gnss_timestamp_ms;
	bool        gnss_valid;

	/* --- Aggregate health --- */
	SmHealth    health;
} SmData;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the sensor manager: I2C buses, all drivers, and
 *          the internal data snapshot.
 * \details Initialises I2C0, I2C1, and each sensor driver in order.
 *          A failure in one sensor is recorded in SmHealth but does not
 *          prevent initialisation of the remaining sensors.
 *
 * \return  SM_OK               if all sensors initialised successfully.
 * \return  SM_ERR_INIT_PARTIAL if at least one sensor failed.
 * \return  SM_ERR_INIT_ALL     if every sensor failed.
 */
SmResult SensorManager_Init(void);

/**
 * \brief   Checks whether the sensor manager has been initialised.
 */
bool SensorManager_IsInitialised(void);

/**
 * \brief   Non-blocking sensor update cycle.
 * \details Must be called every main-loop iteration (ideally every 1–5 ms).
 *          Each sensor is polled only when its config.h sample interval has
 *          elapsed, based on the RP2040 millisecond timer.
 *          PMS5003 is polled every call (byte drain) regardless of interval.
 *
 * \return  SM_OK           always (individual sensor errors tracked in health).
 * \return  SM_ERR_NOT_INIT if called before SensorManager_Init().
 */
SmResult SensorManager_Update(void);

/**
 * \brief   Copies the latest sensor data snapshot to the caller's buffer.
 *
 * \param[out] data   Pointer to receive the copy.
 *
 * \return  SM_OK / SM_ERR_NOT_INIT.
 */
SmResult SensorManager_GetData(SmData *data);

/**
 * \brief   Returns a copy of the current per-sensor health flags.
 *
 * \param[out] health   Pointer to receive the health snapshot.
 *
 * \return  SM_OK / SM_ERR_NOT_INIT.
 */
SmResult SensorManager_GetHealth(SmHealth *health);

/**
 * \brief   Returns the number of sensors that are currently healthy
 *          (initialised and last read succeeded).
 */
uint8_t SensorManager_HealthyCount(void);

#endif /* SENSOR_MANAGER_H_ */
