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
 * File:        particulate_analysis.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   PM-vs-altitude payload analysis module for CONSCIENTIA.
 *
 * Description:
 *   Pairs the latest PMS5003 atmospheric-environment PM concentrations
 *   with the barometric altitude (both AGL from the flight-state module
 *   and MSL from the raw BMP280 reading).
 *
 *   Processing is minimal and intentional:
 *     - The PMS5003 atmospheric fields (pm1_0_atm, pm2_5_atm, pm10_atm)
 *       are used directly — no correction, scaling, or AQI conversion.
 *     - Altitude is taken from two sources: the EMA-smoothed AGL value
 *       passed by the caller (from FsMetrics), and raw MSL from SmData.
 *     - paired_valid is true only when BOTH PM and altitude are fresh
 *       AND their timestamps differ by no more than
 *       PA_TIMESTAMP_STALENESS_MS (config.h).
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
#include "particulate_analysis.h"
#include "config.h"
#include <string.h>
#include <stddef.h>

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool   sInitOk = false;
static PaData sData;

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

PaResult PartAnalysis_Init(void)
{
    sInitOk = false;
    memset(&sData, 0, sizeof(sData));
    sInitOk = true;
    return PA_OK;
}

bool PartAnalysis_IsInitialised(void)
{
    return sInitOk;
}

PaResult PartAnalysis_Update(const SmData *data, float alt_agl)
{
    if (!sInitOk)    { return PA_ERR_NOT_INIT; }
    if (data == NULL) { return PA_ERR_NOT_INIT; }

    /* --- PM data --- */
    sData.pm_valid = data->pm_valid;
    if (data->pm_valid)
    {
        sData.pm1_0_atm       = data->pm.pm1_0_atm;
        sData.pm2_5_atm       = data->pm.pm2_5_atm;
        sData.pm10_atm        = data->pm.pm10_atm;
        sData.pm_timestamp_ms = data->pm_timestamp_ms;
    }

    /* --- Altitude data --- */
    sData.alt_valid = data->baro_valid;
    if (data->baro_valid)
    {
        sData.altitude_m       = alt_agl;
        sData.altitude_msl_m   = data->baro_altitude_m;
        sData.baro_timestamp_ms = data->baro_timestamp_ms;
    }

    /* --- Paired validity (with staleness guard) --- */
    if (sData.pm_valid && sData.alt_valid)
    {
        uint32_t dt = (sData.pm_timestamp_ms >= sData.baro_timestamp_ms)
                    ? (sData.pm_timestamp_ms - sData.baro_timestamp_ms)
                    : (sData.baro_timestamp_ms - sData.pm_timestamp_ms);

        sData.paired_valid = (dt <= PA_TIMESTAMP_STALENESS_MS);
    }
    else
    {
        sData.paired_valid = false;
    }

    return PA_OK;
}

PaResult PartAnalysis_GetData(PaData *out)
{
    if (!sInitOk)  { return PA_ERR_NOT_INIT; }
    if (out == NULL) { return PA_ERR_NOT_INIT; }

    if (!sData.pm_valid && !sData.alt_valid)
    {
        return PA_ERR_NO_DATA;
    }

    *out = sData;
    return PA_OK;
}
