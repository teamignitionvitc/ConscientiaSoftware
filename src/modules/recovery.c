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
 * File:        recovery.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   IMU-based drop-detection and servo deployment for CONSCIENTIA.
 *
 * Description:
 *   Detection algorithm:
 *
 *     1. Each Update() tick, compute |a| = sqrt(ax² + ay² + az²).
 *     2. If |a| < REC_FREEFALL_ACCEL_G (0.4 g), increment the
 *        confirmation counter.  Otherwise, reset it to zero.
 *     3. When the counter reaches REC_FREEFALL_CONFIRM_SAMPLES (3),
 *        call Servo_Deploy() IMMEDIATELY and latch deployed = true.
 *     4. Once latched, Update() becomes a no-op.
 *
 *   Timing budget:
 *     - IMU is sampled at SAMPLE_INTERVAL_IMU_MS = 20 ms.
 *     - At 20 ms sampling, the third consecutive sample arrives
 *       40 ms after the first qualifying sample.
 *     - Servo command is issued on the SAME tick as the third
 *       qualifying sample.
 *     - Confirmation delay: ~40 ms; worst-case release-to-confirmation
 *       is ~60 ms depending on sampling alignment, well below 3000 ms.
 *
 *   Why NO altitude / descent gate:
 *     The CanSat is physically dropped from ~100 m.  There is no
 *     powered ascent phase under CanSat control.  The free-fall
 *     signature is unambiguous and the fastest possible trigger.
 *     Adding altitude or descent requirements would only add latency.
 *
 * \todo  Validate confirmation count and threshold with bench drop tests.
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
#include "recovery.h"
#include "drivers/servo.h"
#include "config.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool      sInitOk = false;
static RecStatus sStatus;

/**************************************************************************************************
 * Static Helpers
 **************************************************************************************************/

static float AccelMag(const Mpu6050Data *imu)
{
    float ax = imu->accel_x_g;
    float ay = imu->accel_y_g;
    float az = imu->accel_z_g;
    return sqrtf(ax * ax + ay * ay + az * az);
}

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

RecResult Recovery_Init(void)
{
    sInitOk = false;
    memset(&sStatus, 0, sizeof(sStatus));
    sStatus.deployed = false;
    sStatus.reason   = REC_REASON_NONE;
    sInitOk = true;
    return REC_OK;
}

bool Recovery_IsInitialised(void)
{
    return sInitOk;
}

RecResult Recovery_Update(const SmData *data, uint32_t now_ms)
{
    if (!sInitOk)    { return REC_ERR_NOT_INIT; }
    if (data == NULL) { return REC_ERR_NOT_INIT; }

    /* --- Latched: nothing more to do --- */
    if (sStatus.deployed)
    {
        return REC_OK;
    }

    /* --- Need valid IMU data --- */
    if (!data->imu_valid)
    {
        sStatus.confirm_count = 0;
        return REC_OK;
    }

    float amag = AccelMag(&data->imu);
    sStatus.accel_mag_g = amag;

    /* --- Free-fall check --- */
    if (amag < REC_FREEFALL_ACCEL_G)
    {
        sStatus.confirm_count++;

        if (sStatus.confirm_count >= REC_FREEFALL_CONFIRM_SAMPLES)
        {
            /* DEPLOY IMMEDIATELY */
            sStatus.detect_time_ms = now_ms;
            (void)Servo_Deploy();
            sStatus.deploy_time_ms = now_ms;
            sStatus.deployed       = true;
            sStatus.reason         = REC_REASON_FREEFALL;
        }
    }
    else
    {
        /* Reset counter — must be consecutive */
        sStatus.confirm_count = 0;
    }

    return REC_OK;
}

bool Recovery_IsDeployed(void)
{
    return sStatus.deployed;
}

RecResult Recovery_GetStatus(RecStatus *status)
{
    if (!sInitOk)     { return REC_ERR_NOT_INIT; }
    if (status == NULL) { return REC_ERR_NOT_INIT; }

    *status = sStatus;
    return REC_OK;
}
