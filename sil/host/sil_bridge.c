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
 * File:        sil_bridge.c
 * Author:      Vidisha Jain
 * Created On:  27/10/26
 *
 * Brief:
 *   Host-side bridge for CONSCIENTIA Software-in-the-Loop testing.
 *
 * Description:
 *   Provides a small C interface around the production FlightState and
 *   Recovery modules so that synthetic sensor samples can be supplied by
 *   the Python SIL test runner.
 *
 *   The production flight algorithms are compiled unchanged. Hardware-only
 *   servo deployment is replaced by servo_mock.c during host testing.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      27/10/26   Vidisha Jain         Initial host SIL bridge
 **************************************************************************************************/

/**************************************************************************************************
 * Includes
 **************************************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "modules/flight_state.h"
#include "modules/recovery.h"
#include "servo_mock.h"


/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

typedef struct
{
    int      phase;

    float    altitude_agl_m;
    float    apogee_m;
    float    vertical_rate_mps;
    float    accel_mag_g;

    bool     recovery_deployed;
    int      recovery_reason;

    uint32_t recovery_detect_time_ms;
    uint32_t recovery_deploy_time_ms;
    uint32_t servo_deploy_count;

} SilResult;


/**************************************************************************************************
 * Public Functions
 **************************************************************************************************/

void Sil_Init(void)
{
    SilServoMock_Reset();

    (void)FlightState_Init();
    (void)Recovery_Init();
}


void Sil_Update(
    float altitude_m,
    bool baro_valid,
    float accel_x_g,
    float accel_y_g,
    float accel_z_g,
    bool imu_valid,
    uint32_t now_ms)
{
    SmData data;

    memset(&data, 0, sizeof(data));

    data.baro_altitude_m    = altitude_m;
    data.baro_timestamp_ms  = now_ms;
    data.baro_valid         = baro_valid;

    data.imu.accel_x_g      = accel_x_g;
    data.imu.accel_y_g      = accel_y_g;
    data.imu.accel_z_g      = accel_z_g;
    data.imu_timestamp_ms   = now_ms;
    data.imu_valid          = imu_valid;

    /*
     * Keep production execution order identical to main.c:
     *
     *     FlightState_Update()
     *     Recovery_Update()
     */
    (void)FlightState_Update(&data, now_ms);
    (void)Recovery_Update(&data, now_ms);
}


void Sil_GetResult(SilResult *result)
{
    FsMetrics fs;
    RecStatus rec;

    if (result == NULL)
    {
        return;
    }

    memset(result, 0, sizeof(*result));
    memset(&fs, 0, sizeof(fs));
    memset(&rec, 0, sizeof(rec));

    (void)FlightState_GetMetrics(&fs);
    (void)Recovery_GetStatus(&rec);

    result->phase                 = (int)fs.phase;

    result->altitude_agl_m        = fs.alt_agl_m;
    result->apogee_m              = fs.apogee_m;
    result->vertical_rate_mps     = fs.vert_rate_mps;
    result->accel_mag_g           = fs.accel_mag_g;

    result->recovery_deployed     = rec.deployed;
    result->recovery_reason       = (int)rec.reason;

    result->recovery_detect_time_ms = rec.detect_time_ms;
    result->recovery_deploy_time_ms = rec.deploy_time_ms;

    result->servo_deploy_count =
        SilServoMock_GetDeployCount();
}