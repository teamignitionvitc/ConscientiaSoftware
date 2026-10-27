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
                         .::::          ::          ::::
                       .::::::::.       ::       .:::::::::
                       ::::::::::::....::::.....:::::::::::
                        .:::::::::::::::::::::::::::::::::.

                    © Copyright of Ignition Software Department
*/
/**************************************************************************************************
 * File:        sensor_manager.h
 * Author:      Vidisha Jain
 * Created On:  27/10/26
 *
 * \brief       Minimal host-only SensorManager type stub for SIL compilation.
 * \details     Defines only the sensor data structures required by production
 *              flight-state and recovery modules. This file is never used by
 *              the RP2040 firmware build and contains no hardware behaviour.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      27/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef SENSOR_MANAGER_H_
#define SENSOR_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * Host-side replacement for modules/sensor_manager.h.
 *
 * Contains only the data required by the production FlightState and
 * Recovery modules during SIL execution. No RP2040 hardware dependencies.
 */

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float temperature_c;
} Mpu6050Data;


typedef struct
{
    float       baro_altitude_m;
    uint32_t    baro_timestamp_ms;
    bool        baro_valid;

    Mpu6050Data imu;
    uint32_t    imu_timestamp_ms;
    bool        imu_valid;

} SmData;

#endif /* SENSOR_MANAGER_H_ */