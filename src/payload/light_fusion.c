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
 * File:        light_fusion.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Multi-sensor light fusion payload module for CONSCIENTIA.
 *
 * Description:
 *   Fusion method (PROVISIONAL):
 *   ─────────────────────────────
 *   Simple equal-weight arithmetic mean of all VALID BH1750 readings.
 *
 *       fused_lux = Σ valid_lux[i] / N_valid
 *
 *   If only one sensor is valid, fused_lux = that sensor.
 *   If zero sensors are valid, fused_valid = false.
 *
 *   Quality metric:
 *     spread_lux = max(valid) − min(valid)
 *
 *   Why provisional:
 *     The three BH1750 sensors are mounted at different positions on the
 *     CanSat body.  During descent under rotation, each sees a different
 *     solid angle of the sky.  A proper fusion would weight each sensor
 *     by its viewing geometry or reject outliers using a median filter.
 *     That requires flight characterisation data we don't have yet.
 *
 *   \todo  Replace equal-weight mean with a weighted or median scheme once
 *          flight data reveals the typical spread between sensors.
 *   \todo  Use IMU gyro Z-axis rate to detect full rotations and apply a
 *          rolling average over one complete rotation for smoother output.
 *
 *   Motion hint:
 *     LfMotionHint is accepted and stored in the output struct for
 *     downstream consumers (graphing, telemetry) to contextualise the
 *     reading quality.  The fusion algorithm itself does NOT change
 *     behaviour based on motion — that is left for the post-flight
 *     analysis revision.
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
#include "light_fusion.h"
#include <string.h>
#include <stddef.h>

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool   sInitOk = false;
static LfData sData;

/**************************************************************************************************
 * Static Helpers
 **************************************************************************************************/

static inline float MinF(float a, float b) { return (a < b) ? a : b; }
static inline float MaxF(float a, float b) { return (a > b) ? a : b; }

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

LfResult LightFusion_Init(void)
{
    sInitOk = false;
    memset(&sData, 0, sizeof(sData));
    sInitOk = true;
    return LF_OK;
}

bool LightFusion_IsInitialised(void)
{
    return sInitOk;
}

LfResult LightFusion_Update(const SmData *data, float alt_agl,
                            LfMotionHint motion)
{
    if (!sInitOk)    { return LF_ERR_NOT_INIT; }
    if (data == NULL) { return LF_ERR_NOT_INIT; }

    /* --- Preserve raw per-sensor values --- */
    sData.lux1   = data->light1_lux;
    sData.lux2   = data->light2_lux;
    sData.lux3   = data->light3_lux;
    sData.valid1 = data->light1_valid;
    sData.valid2 = data->light2_valid;
    sData.valid3 = data->light3_valid;

    sData.light_timestamp_ms = data->light_timestamp_ms;

    /* --- Context --- */
    sData.motion = motion;

    /* --- Altitude --- */
    sData.alt_valid = data->baro_valid;
    if (data->baro_valid)
    {
        sData.altitude_m      = alt_agl;
        sData.baro_timestamp_ms = data->baro_timestamp_ms;
    }

    /* --- Fusion: equal-weight mean of valid sensors (PROVISIONAL) --- */
    /*
     * TODO: Replace with weighted / median scheme once characterised.
     *       Sensor weights are intentionally NOT locked to 1/N here.
     */
    float  sum   = 0.0f;
    float  vmin  = 1.0e9f;
    float  vmax  = 0.0f;
    uint8_t nval = 0;

    if (sData.valid1)
    {
        sum  += sData.lux1;
        vmin  = MinF(vmin, sData.lux1);
        vmax  = MaxF(vmax, sData.lux1);
        nval++;
    }
    if (sData.valid2)
    {
        sum  += sData.lux2;
        vmin  = MinF(vmin, sData.lux2);
        vmax  = MaxF(vmax, sData.lux2);
        nval++;
    }
    if (sData.valid3)
    {
        sum  += sData.lux3;
        vmin  = MinF(vmin, sData.lux3);
        vmax  = MaxF(vmax, sData.lux3);
        nval++;
    }

    sData.sensors_used = nval;

    if (nval > 0)
    {
        sData.fused_lux   = sum / (float)nval;
        sData.spread_lux  = vmax - vmin;
        sData.fused_valid = true;
    }
    else
    {
        sData.fused_lux   = 0.0f;
        sData.spread_lux  = 0.0f;
        sData.fused_valid = false;
    }

    return LF_OK;
}

LfResult LightFusion_GetData(LfData *out)
{
    if (!sInitOk)  { return LF_ERR_NOT_INIT; }
    if (out == NULL) { return LF_ERR_NOT_INIT; }

    if (!sData.fused_valid && !sData.alt_valid)
    {
        return LF_ERR_NO_DATA;
    }

    *out = sData;
    return LF_OK;
}
