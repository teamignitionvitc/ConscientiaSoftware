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
 * File:        wind_inference.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Post-flight wind inference payload module for CONSCIENTIA.
 *
 * Description:
 *   Records altitude-tagged horizontal GNSS velocity and derives a
 *   provisional first-order wind estimate.
 *
 *   Equations
 *   ─────────
 *   1. Ground velocity decomposition (GNSS course uses true-north CW):
 *
 *        V_north = speed * cos(course)
 *        V_east  = speed * sin(course)
 *
 *   2. Provisional wind estimate (descent-only assumption):
 *
 *        W_speed ≈ gnd_speed                     (magnitude)
 *        W_dir   = course + 180° (mod 360)       (wind comes FROM)
 *        W_north = −V_north     = −speed * cos(course)
 *        W_east  = −V_east      = −speed * sin(course)
 *
 *      Rationale: under parachute with negligible self-propulsion,
 *      the CanSat drifts WITH the wind.  Ground motion IS the wind.
 *      The wind vector points opposite to the direction the CanSat
 *      is moving — the wind is coming FROM the opposite direction.
 *
 *      Wait — actually that's wrong.  If the CanSat drifts WITH the
 *      wind, then the CanSat's ground track direction is the direction
 *      the wind blows TO.  Meteorological convention defines wind
 *      direction as the direction FROM which the wind blows.
 *      So:
 *        W_dir   = (course + 180°) mod 360°      (FROM direction)
 *        W_north = V_north  (wind blows TO the same direction)
 *        W_east  = V_east
 *        W_speed = gnd_speed
 *
 *      The wind velocity components equal the ground velocity
 *      components (same direction and magnitude).  Only the
 *      FROM-direction label is inverted by +180°.
 *
 *   3. Motion quality (IMU gyro):
 *
 *        gyro_mag = sqrt(gx² + gy² + gz²)   [dps]
 *
 *        GOOD:    gyro_mag < WI_GYRO_GOOD_DPS     (< 30 dps)
 *        FAIR:    gyro_mag < WI_GYRO_FAIR_DPS     (< 90 dps)
 *        POOR:    gyro_mag ≥ WI_GYRO_FAIR_DPS
 *
 *   Assumptions & limitations
 *   ─────────────────────────
 *   - Wind ≈ ground velocity is valid ONLY during parachute descent.
 *     During ascent (powered carrier) or on the ground, the estimate
 *     is meaningless.  The caller must gate by flight state.
 *   - No parachute aerodynamic drag correction is applied.
 *   - L89HA GNSS ground speed at very low velocities (<0.3 m/s) is
 *     dominated by receiver noise; a minimum speed gate rejects
 *     these samples (WI_MIN_SPEED_MPS in config.h).
 *   - Course is undefined when speed ≈ 0; the driver may report
 *     arbitrary values.  Rejected alongside speed.
 *   - IMU does NOT provide absolute heading — gyro magnitude is used
 *     only for a quality label.
 *
 * \todo  Add configurable parachute drag model for improved accuracy.
 * \todo  Post-flight ground-station tool should aggregate these samples
 *        into altitude-binned wind profiles.
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
#include "wind_inference.h"
#include "config.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

/**************************************************************************************************
 * Internal Constants
 *
 * Kept here rather than config.h because they are specific to this module's
 * implementation and unlikely to be shared.  Move to config.h if the team
 * prefers full centralisation.
 **************************************************************************************************/

/** Minimum GNSS ground speed (m/s) to consider a sample usable.
 *  Below this, receiver noise dominates and course is unreliable. */
#define WI_MIN_SPEED_MPS        (0.3f)

/** Gyro magnitude threshold for GOOD quality (°/s). */
#define WI_GYRO_GOOD_DPS        (30.0f)

/** Gyro magnitude threshold for FAIR quality (°/s). */
#define WI_GYRO_FAIR_DPS        (90.0f)

/** Degrees-to-radians factor. */
#define WI_DEG2RAD              (0.01745329251994f)   /* π / 180 */

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool   sInitOk    = false;
static bool   sHasData   = false;
static WiData sData;

/**************************************************************************************************
 * Static Helpers
 **************************************************************************************************/

/**
 * \brief   Wrap angle to [0, 360).
 */
static float WrapDeg(float deg)
{
    while (deg < 0.0f)   { deg += 360.0f; }
    while (deg >= 360.0f) { deg -= 360.0f; }
    return deg;
}

/**
 * \brief   Compute gyroscope vector magnitude.
 */
static float GyroMag(const Mpu6050Data *imu)
{
    float gx = imu->gyro_x_dps;
    float gy = imu->gyro_y_dps;
    float gz = imu->gyro_z_dps;
    return sqrtf(gx * gx + gy * gy + gz * gz);
}

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

WiResult WindInfer_Init(void)
{
    sInitOk  = false;
    sHasData = false;
    memset(&sData, 0, sizeof(sData));
    sInitOk = true;
    return WI_OK;
}

bool WindInfer_IsInitialised(void)
{
    return sInitOk;
}

WiResult WindInfer_Update(const SmData *data, float alt_agl, uint32_t now_ms)
{
    if (!sInitOk)    { return WI_ERR_NOT_INIT; }
    if (data == NULL) { return WI_ERR_NOT_INIT; }

    /* --- GNSS validity --- */
    bool gnss_ok = data->gnss_valid
                && data->gnss.fix_valid
                && (data->gnss.speed_mps >= WI_MIN_SPEED_MPS);

    sData.gnss_valid = gnss_ok;

    if (gnss_ok)
    {
        float speed  = data->gnss.speed_mps;
        float course = data->gnss.course_deg;
        float rad    = course * WI_DEG2RAD;

        /* --- Measured ground velocity --- */
        sData.gnd_speed_mps  = speed;
        sData.gnd_course_deg = course;
        sData.gnd_vn_mps     = speed * cosf(rad);
        sData.gnd_ve_mps     = speed * sinf(rad);
        sData.satellites      = data->gnss.satellites;

        /* --- Inferred wind estimate (PROVISIONAL) --- */
        /*
         * Under parachute: CanSat drifts WITH the wind.
         * Wind blows in the same direction as the ground track.
         * Meteorological "from" direction is opposite.
         *
         * Wind velocity components = ground velocity components.
         * Wind FROM direction = course + 180°.
         */
        sData.wind_speed_mps = speed;
        sData.wind_dir_deg   = WrapDeg(course + 180.0f);
        sData.wind_vn_mps    = sData.gnd_vn_mps;
        sData.wind_ve_mps    = sData.gnd_ve_mps;
    }

    /* --- Altitude --- */
    sData.alt_valid = data->baro_valid;
    if (data->baro_valid)
    {
        sData.altitude_agl_m = alt_agl;
        sData.altitude_msl_m = data->baro_altitude_m;
    }

    /* --- IMU motion-quality tier --- */
    if (data->imu_valid)
    {
        float gmag = GyroMag(&data->imu);
        sData.gyro_mag_dps = gmag;

        if (gmag < WI_GYRO_GOOD_DPS)
        {
            sData.quality = WI_QUALITY_GOOD;
        }
        else if (gmag < WI_GYRO_FAIR_DPS)
        {
            sData.quality = WI_QUALITY_FAIR;
        }
        else
        {
            sData.quality = WI_QUALITY_POOR;
        }
    }
    else
    {
        sData.quality      = WI_QUALITY_UNKNOWN;
        sData.gyro_mag_dps = 0.0f;
    }

    /* --- Combined validity --- */
    sData.sample_valid = (gnss_ok && data->baro_valid);
    sData.timestamp_ms = now_ms;

    if (sData.sample_valid)
    {
        sHasData = true;
    }

    return WI_OK;
}

WiResult WindInfer_GetData(WiData *out)
{
    if (!sInitOk)  { return WI_ERR_NOT_INIT; }
    if (out == NULL) { return WI_ERR_NOT_INIT; }

    if (!sHasData)
    {
        return WI_ERR_NO_DATA;
    }

    *out = sData;
    return WI_OK;
}
