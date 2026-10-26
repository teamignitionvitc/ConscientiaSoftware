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

                    © Copyright of Ignition Avionics Department
*/

/**************************************************************************************************
 * File:        mpu6050.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Header file for MPU6050 6-axis IMU driver.
 * \details     Provides the public API for initializing, configuring, and reading
 *              the InvenSense MPU6050 accelerometer/gyroscope over I2C on the
 *              Waveshare RP2040-Zero.  This is a hardware driver only — no flight
 *              logic, recovery detection, or telemetry.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef MPU6050_H_
#define MPU6050_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * Macro Definitions
 **************************************************************************************************/

/** \brief  Expected WHO_AM_I value read from register 0x75. */
#define MPU6050_WHO_AM_I_VAL            (0x68U)

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by MPU6050 driver functions.
 */
typedef enum
{
	MPU6050_OK = 0,             /**< Operation succeeded.                   */
	MPU6050_ERR_I2C,            /**< I2C read/write failure.                */
	MPU6050_ERR_WHO_AM_I,       /**< Unexpected WHO_AM_I (not 0x68).        */
	MPU6050_ERR_NOT_INIT        /**< Driver used before successful init.    */
} Mpu6050Result;

/**
 * \brief   Raw 16-bit signed readings from one 3-axis sensor.
 */
typedef struct
{
	int16_t		x;
	int16_t		y;
	int16_t		z;
} Mpu6050RawAxes;

/**
 * \brief   Converted accelerometer readings in g.
 */
typedef struct
{
	float		x_g;
	float		y_g;
	float		z_g;
} Mpu6050AccelData;

/**
 * \brief   Converted gyroscope readings in degrees per second.
 */
typedef struct
{
	float		x_dps;
	float		y_dps;
	float		z_dps;
} Mpu6050GyroData;

/**
 * \brief   Combined converted IMU measurement.
 */
typedef struct
{
	float		accel_x_g;
	float		accel_y_g;
	float		accel_z_g;

	float		gyro_x_dps;
	float		gyro_y_dps;
	float		gyro_z_dps;

	float		temperature_c;
} Mpu6050Data;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the MPU6050 driver.
 * \details Verifies WHO_AM_I, wakes the device from sleep, selects the
 *          PLL clock source, and writes the initial range/filter config.
 *          The I2C bus must already be initialised by the caller before
 *          calling this function.
 *
 * \return  MPU6050_OK            on success.
 * \return  MPU6050_ERR_I2C       if I2C communication fails.
 * \return  MPU6050_ERR_WHO_AM_I  if the identity register does not read 0x68.
 */
Mpu6050Result MPU6050_Init(void);

/**
 * \brief   Checks whether the driver has been initialised successfully.
 *
 * \return  true if MPU6050_Init() completed without error, false otherwise.
 */
bool MPU6050_IsInitialised(void);

/**
 * \brief   Reads raw 16-bit accelerometer values.
 *
 * \param[out] raw   Pointer to receive raw X/Y/Z accelerometer counts.
 *
 * \return  MPU6050_OK            on success.
 * \return  MPU6050_ERR_NOT_INIT  if the driver is not initialised.
 * \return  MPU6050_ERR_I2C       if I2C communication fails.
 */
Mpu6050Result MPU6050_ReadRawAccel(Mpu6050RawAxes *raw);

/**
 * \brief   Reads raw 16-bit gyroscope values.
 *
 * \param[out] raw   Pointer to receive raw X/Y/Z gyroscope counts.
 *
 * \return  MPU6050_OK            on success.
 * \return  MPU6050_ERR_NOT_INIT  if the driver is not initialised.
 * \return  MPU6050_ERR_I2C       if I2C communication fails.
 */
Mpu6050Result MPU6050_ReadRawGyro(Mpu6050RawAxes *raw);

/**
 * \brief   Reads converted accelerometer values in g.
 *
 * \param[out] accel   Pointer to receive X/Y/Z acceleration in g.
 *
 * \return  MPU6050_OK            on success.
 * \return  MPU6050_ERR_NOT_INIT  if the driver is not initialised.
 * \return  MPU6050_ERR_I2C       if I2C communication fails.
 */
Mpu6050Result MPU6050_ReadAccel(Mpu6050AccelData *accel);

/**
 * \brief   Reads converted gyroscope values in degrees/second.
 *
 * \param[out] gyro   Pointer to receive X/Y/Z angular rate in dps.
 *
 * \return  MPU6050_OK            on success.
 * \return  MPU6050_ERR_NOT_INIT  if the driver is not initialised.
 * \return  MPU6050_ERR_I2C       if I2C communication fails.
 */
Mpu6050Result MPU6050_ReadGyro(Mpu6050GyroData *gyro);

/**
 * \brief   Reads the MPU6050 die temperature.
 *
 * \param[out] temperature_c   Pointer to receive temperature in °C.
 *
 * \return  MPU6050_OK            on success.
 * \return  MPU6050_ERR_NOT_INIT  if the driver is not initialised.
 * \return  MPU6050_ERR_I2C       if I2C communication fails.
 */
Mpu6050Result MPU6050_ReadTemperature(float *temperature_c);

/**
 * \brief   Reads all sensors (accel + gyro + temperature) in a single burst.
 * \details All 14 data bytes (accel[6], temp[2], gyro[6]) are read in one
 *          I2C transaction for temporal consistency.
 *
 * \param[out] data   Pointer to Mpu6050Data struct to receive all readings.
 *
 * \return  MPU6050_OK            on success.
 * \return  MPU6050_ERR_NOT_INIT  if the driver is not initialised.
 * \return  MPU6050_ERR_I2C       if I2C communication fails.
 */
Mpu6050Result MPU6050_Read(Mpu6050Data *data);

#endif /* MPU6050_H_ */
