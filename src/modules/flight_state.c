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
 * File:        flight_state.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Flight-phase state machine for CONSCIENTIA.
 *
 * Description:
 *   State machine with four phases:
 *
 *     PRE_LAUNCH ──► ASCENT ──► DESCENT ──► LANDED
 *
 *   Altitude processing
 *   ───────────────────
 *   1. Raw barometric altitude from BMP280 (baro_altitude_m in SmData) is
 *      passed through a simple EMA low-pass filter (α from config.h).
 *   2. The first valid baro reading after Init() is latched as the launch-
 *      pad reference.  All subsequent altitudes are converted to AGL.
 *   3. Vertical rate is estimated from successive filtered altitude samples
 *      with the elapsed time between baro updates (≈ SAMPLE_INTERVAL_BARO_MS).
 *
 *   Transition guards
 *   ─────────────────
 *   Every transition requires N consecutive qualifying baro samples to pass
 *   a confirmation counter, where N is a per-transition constant in config.h.
 *   This eliminates false triggers from single-sample noise spikes.
 *
 *   IMU as supporting evidence
 *   ──────────────────────────
 *   The MPU6050 accelerometer magnitude |a| is computed each update.
 *   - LANDED check requires |a| ≈ 1 g  (within FS_LANDED_ACCEL_BAND_G).
 *   - Future: launch-detection impulse confirmation (not yet implemented).
 *
 *   This module does NOT trigger a servo or any recovery deployment.
 *
 * \todo  Add IMU-based launch-kick confirmation for PRE_LAUNCH → ASCENT.
 * \todo  Validate thresholds with actual flight data.
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
#include "flight_state.h"
#include "config.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool        sInitOk         = false;
static bool        sRefLatched     = false;   /* true once launch-pad alt is captured */

static FsMetrics   sMetrics;

/* EMA-filtered altitude (MSL, metres) */
static float       sAltSmoothed    = 0.0f;

/* Previous filtered altitude + timestamp for vertical-rate estimation */
static float       sAltPrev        = 0.0f;
static uint32_t    sBaroPrevMs     = 0;

/* Confirmation counters (count consecutive qualifying samples) */
static uint8_t     sAscentCount    = 0;
static uint8_t     sDescentCount   = 0;
static uint8_t     sLandedCount    = 0;

/* Timestamp at which the current state was entered */
static uint32_t    sStateEntryMs   = 0;

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/
static void   ProcessAltitude(const SmData *data, uint32_t now_ms);
static void   ProcessImu(const SmData *data);
static void   EvalPreLaunch(uint32_t now_ms);
static void   EvalAscent(uint32_t now_ms);
static void   EvalDescent(uint32_t now_ms);
static float  AccelMag(const Mpu6050Data *imu);

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

FsResult FlightState_Init(void)
{
    sInitOk     = false;
    sRefLatched = false;

    memset(&sMetrics, 0, sizeof(sMetrics));
    sMetrics.phase = FS_PRE_LAUNCH;

    sAltSmoothed   = 0.0f;
    sAltPrev       = 0.0f;
    sBaroPrevMs    = 0;

    sAscentCount   = 0;
    sDescentCount  = 0;
    sLandedCount   = 0;
    sStateEntryMs  = 0;

    sInitOk = true;
    return FS_OK;
}

bool FlightState_IsInitialised(void)
{
    return sInitOk;
}

FsResult FlightState_Update(const SmData *data, uint32_t now_ms)
{
    if (!sInitOk)   { return FS_ERR_NOT_INIT; }
    if (data == NULL) { return FS_ERR_NOT_INIT; }

    /* --- Process sensors --- */
    ProcessAltitude(data, now_ms);
    ProcessImu(data);

    /* --- State evaluation --- */
    switch (sMetrics.phase)
    {
        case FS_PRE_LAUNCH: EvalPreLaunch(now_ms); break;
        case FS_ASCENT:     EvalAscent(now_ms);    break;
        case FS_DESCENT:    EvalDescent(now_ms);   break;
        case FS_LANDED:     /* Terminal state */    break;
        default:            /* Should not happen */ break;
    }

    sMetrics.state_age_ms = now_ms - sStateEntryMs;

    return FS_OK;
}

FlightPhase FlightState_GetPhase(void)
{
    return sMetrics.phase;
}

FsResult FlightState_GetMetrics(FsMetrics *metrics)
{
    if (!sInitOk)      { return FS_ERR_NOT_INIT; }
    if (metrics == NULL) { return FS_ERR_NOT_INIT; }

    *metrics = sMetrics;
    return FS_OK;
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

/**
 * \brief   EMA-filter the baro altitude, compute AGL, update vertical rate
 *          and apogee.
 */
static void ProcessAltitude(const SmData *data, uint32_t now_ms)
{
    if (!data->baro_valid)
    {
        return;     /* No new baro data — keep previous estimates */
    }

    float raw_alt = data->baro_altitude_m;

    /* Sanity-cap */
    if (raw_alt > FS_MAX_ALTITUDE_M || raw_alt < -100.0f)
    {
        return;
    }

    /* --- Latch launch-pad reference on the first valid sample --- */
    if (!sRefLatched)
    {
        sMetrics.launch_alt_m = raw_alt;
        sAltSmoothed          = raw_alt;
        sAltPrev              = raw_alt;
        sBaroPrevMs           = now_ms;
        sRefLatched           = true;
        sStateEntryMs         = now_ms;
        return;
    }

    /* --- EMA low-pass filter --- */
    sAltSmoothed = FS_ALT_EMA_ALPHA * raw_alt
                 + (1.0f - FS_ALT_EMA_ALPHA) * sAltSmoothed;

    /* --- AGL (above ground level = above launch pad) --- */
    sMetrics.alt_agl_m = sAltSmoothed - sMetrics.launch_alt_m;

    /* --- Vertical rate (m/s, positive = upward) --- */
    uint32_t dt_ms = now_ms - sBaroPrevMs;
    if (dt_ms > 0)
    {
        float dt_s = (float)dt_ms / 1000.0f;
        sMetrics.vert_rate_mps = (sAltSmoothed - sAltPrev) / dt_s;
    }

    sAltPrev    = sAltSmoothed;
    sBaroPrevMs = now_ms;

    /* --- Track apogee --- */
    if (sMetrics.alt_agl_m > sMetrics.apogee_m)
    {
        sMetrics.apogee_m = sMetrics.alt_agl_m;
    }
}

/**
 * \brief   Compute accelerometer magnitude and store in metrics.
 */
static void ProcessImu(const SmData *data)
{
    if (!data->imu_valid)
    {
        return;
    }

    sMetrics.accel_mag_g = AccelMag(&data->imu);
}

/**
 * \brief   PRE_LAUNCH → ASCENT evaluation.
 * \details Transition when altitude AGL exceeds FS_ASCENT_ALT_THRESHOLD_M
 *          for FS_ASCENT_CONFIRM_SAMPLES consecutive baro readings.
 */
static void EvalPreLaunch(uint32_t now_ms)
{
    if (!sRefLatched) { return; }

    if (sMetrics.alt_agl_m >= FS_ASCENT_ALT_THRESHOLD_M)
    {
        sAscentCount++;
        if (sAscentCount >= FS_ASCENT_CONFIRM_SAMPLES)
        {
            sMetrics.phase          = FS_ASCENT;
            sMetrics.ascent_time_ms = now_ms;
            sStateEntryMs           = now_ms;
            sAscentCount            = 0;
            sDescentCount           = 0;
            sLandedCount            = 0;
        }
    }
    else
    {
        sAscentCount = 0;
    }
}

/**
 * \brief   ASCENT → DESCENT evaluation.
 * \details Transition when:
 *          1. Altitude has dropped at least FS_DESCENT_ALT_THRESHOLD_M
 *             below the recorded apogee, AND
 *          2. Vertical rate is negative (falling) with magnitude ≥
 *             FS_DESCENT_RATE_MIN_MPS.
 *          Both conditions must hold for FS_DESCENT_CONFIRM_SAMPLES.
 */
static void EvalAscent(uint32_t now_ms)
{
    float drop_from_apogee = sMetrics.apogee_m - sMetrics.alt_agl_m;

    bool alt_qualifies  = (drop_from_apogee >= FS_DESCENT_ALT_THRESHOLD_M);
    bool rate_qualifies = (sMetrics.vert_rate_mps <= -FS_DESCENT_RATE_MIN_MPS);

    if (alt_qualifies && rate_qualifies)
    {
        sDescentCount++;
        if (sDescentCount >= FS_DESCENT_CONFIRM_SAMPLES)
        {
            sMetrics.phase           = FS_DESCENT;
            sMetrics.descent_time_ms = now_ms;
            sStateEntryMs            = now_ms;
            sDescentCount            = 0;
            sLandedCount             = 0;
        }
    }
    else
    {
        sDescentCount = 0;
    }
}

/**
 * \brief   DESCENT → LANDED evaluation.
 * \details Transition when BOTH conditions hold simultaneously for
 *          FS_LANDED_HOLD_SAMPLES consecutive baro ticks:
 *          1. |vertical rate| < FS_LANDED_VRATE_THRESH_MPS
 *          2. |accel_mag − 1 g| < FS_LANDED_ACCEL_BAND_G
 */
static void EvalDescent(uint32_t now_ms)
{
    float abs_vrate = fabsf(sMetrics.vert_rate_mps);
    float accel_err = fabsf(sMetrics.accel_mag_g - 1.0f);

    bool vrate_settled = (abs_vrate < FS_LANDED_VRATE_THRESH_MPS);
    bool accel_settled = (accel_err < FS_LANDED_ACCEL_BAND_G);

    if (vrate_settled && accel_settled)
    {
        sLandedCount++;
        if (sLandedCount >= FS_LANDED_HOLD_SAMPLES)
        {
            sMetrics.phase          = FS_LANDED;
            sMetrics.landed_time_ms = now_ms;
            sStateEntryMs           = now_ms;
            sLandedCount            = 0;
        }
    }
    else
    {
        sLandedCount = 0;
    }
}

/**
 * \brief   Computes the Euclidean magnitude of the accelerometer vector.
 */
static float AccelMag(const Mpu6050Data *imu)
{
    float ax = imu->accel_x_g;
    float ay = imu->accel_y_g;
    float az = imu->accel_z_g;

    return sqrtf(ax * ax + ay * ay + az * az);
}
