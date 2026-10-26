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
 * File:        air_quality.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Source file for the MQ135 air-quality processing module (payload layer).
 *
 * Description:
 *   Converts raw MQ135 sensor voltage (obtained from the MQ135 hardware
 *   driver) into the sensor resistance Rs, the Rs/R0 ratio, and a bounded
 *   relative air-quality response value.
 *
 *   Rs calculation:
 *     The MQ135 internal resistance Rs is in series with the load resistor
 *     RL between Vcc (5 V) and GND.  The voltage at the junction is
 *     sensor_voltage_mv (supplied by the driver after divider correction).
 *
 *       Rs = RL * (Vcc_mv - V_sensor_mv) / V_sensor_mv
 *
 *   Rs/R0:
 *     R0 is the sensor resistance in clean air (provisionally set to
 *     MQ135_R0_OHMS_PROVISIONAL in config.h — see calibration TODO).
 *
 *   Air-quality response:
 *       response = clamp(100.0 / rs_over_r0, 0.0, 100.0)
 *
 *     In clean air Rs/R0 is high, so response is low.
 *     In polluted air Rs/R0 drops, so response rises toward 100.
 *
 * \warning  MQ135_R0_OHMS_PROVISIONAL is NOT laboratory-calibrated.
 *           Response values are relative and provisional until R0 is
 *           determined through a formal clean-air calibration procedure.
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
#include "air_quality.h"
#include "drivers/mq135.h"
#include "config.h"

#include <stddef.h>

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/

/** MQ135 supply voltage (mV). The sensor heater circuit runs at 5 V. */
#define MQ135_VCC_MV        (5000.0f)

/** Load resistor from config.h, cast to float for calculations. */
#define RL_F                ((float)(MQ135_RL_OHMS))

/** Provisional R0 from config.h, cast to float for calculations. */
#define R0_F                ((float)(MQ135_R0_OHMS_PROVISIONAL))

/** Clamp helper — inline to avoid function-call overhead. */
#define CLAMP_F(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool sInitOk = false;

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

AqResult AirQuality_Init(void)
{
    sInitOk = false;

    if (!MQ135_IsInitialised())
    {
        return AQ_ERR_NOT_INIT;
    }

    sInitOk = true;
    return AQ_OK;
}

bool AirQuality_IsInitialised(void)
{
    return sInitOk;
}

AqResult AirQuality_Read(AqData *data)
{
    Mq135Data   hw;
    Mq135Result hw_res;
    float       rs;
    float       rs_over_r0;

    if (!sInitOk)
    {
        return AQ_ERR_NOT_INIT;
    }
    if (data == NULL)
    {
        return AQ_ERR_INVALID_INPUT;
    }

    /* Read hardware driver */
    hw_res = MQ135_Read(&hw);
    if (hw_res != MQ135_OK)
    {
        return AQ_ERR_NOT_INIT;
    }

    /* Guard against zero sensor voltage (ADC reads nothing / disconnected) */
    if (hw.sensor_voltage_mv <= 0.0f)
    {
        data->valid = false;
        return AQ_ERR_INVALID_INPUT;
    }

    /*
     * Rs calculation:
     *   Rs = RL * (Vcc - V_sensor) / V_sensor
     *
     * If V_sensor >= Vcc the expression would go negative (sensor shorted
     * or heavily saturated), clamp Rs to RL as a floor.
     */
    if (hw.sensor_voltage_mv >= MQ135_VCC_MV)
    {
        rs = RL_F;
    }
    else
    {
        rs = RL_F * (MQ135_VCC_MV - hw.sensor_voltage_mv) / hw.sensor_voltage_mv;
    }

    /*
     * Rs/R0 — guard against division by zero (should never happen with a
     * valid R0 constant, but protects at runtime regardless).
     */
    rs_over_r0 = (R0_F > 0.0f) ? (rs / R0_F) : 1.0f;

    /*
     * Relative air-quality response:
     *   response = clamp(100.0 / rs_over_r0, 0.0, 100.0)
     *
     * rs_over_r0 > 1  (cleaner than baseline)  → response < 100
     * rs_over_r0 = 1  (at baseline Rs = R0)    → response = 100
     * rs_over_r0 < 1  (heavier pollution load)  → response clamped to 100
     * rs_over_r0 → ∞  (very clean / near open)  → response → 0
     */
    float response = CLAMP_F(100.0f / rs_over_r0, 0.0f, 100.0f);

    /* Populate output */
    data->raw_adc              = hw.raw_adc;
    data->adc_voltage_mv       = hw.adc_voltage_mv;
    data->sensor_voltage_mv    = hw.sensor_voltage_mv;
    data->rs_ohms              = rs;
    data->rs_over_r0           = rs_over_r0;
    data->air_quality_response = response;
    data->valid                = true;

    return AQ_OK;
}
