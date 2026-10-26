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
 * File:        mq135.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Source file for the MQ135 air quality sensor driver.
 *
 * Description:
 *   Implements RP2040 ADC initialisation and raw/voltage readout for the
 *   MQ135 analog gas sensor.  The sensor's 5 V AOUT is scaled to 3.3 V
 *   via an external voltage divider (R1 = 33 kΩ, R2 = 56 kΩ) and read on
 *   ADC channel 0 (GPIO26) as defined in config.h.
 *
 *   This driver does NOT implement gas-concentration calibration or AQI
 *   calculation.  Those responsibilities belong in the payload / module
 *   layer once a defensible calibration method is established.
 *
 * \todo    Establish clean-air baseline calibration methodology.
 * \todo    Evaluate whether Rs/Ro resistance-ratio conversion is worthwhile
 *          given the MQ135's broad sensitivity and environmental sensitivity.
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
#include "mq135.h"
#include "config.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/

/**
 * \brief   Voltage-divider gain to reconstruct the sensor output voltage.
 * \details V_sensor = V_adc * (R1 + R2) / R2
 */
#define DIVIDER_GAIN    (((float)(MQ135_DIVIDER_R1_OHMS) + \
                          (float)(MQ135_DIVIDER_R2_OHMS)) / \
                         (float)(MQ135_DIVIDER_R2_OHMS))

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool sInitOk = false;

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

Mq135Result MQ135_Init(void)
{
    sInitOk = false;

    adc_init();
    adc_gpio_init(PIN_MQ135_ADC);
    adc_select_input(MQ135_ADC_CHANNEL);

    sInitOk = true;
    return MQ135_OK;
}

bool MQ135_IsInitialised(void)
{
    return sInitOk;
}

Mq135Result MQ135_ReadRaw(uint16_t *raw_out)
{
    if (!sInitOk)
    {
        return MQ135_ERR_NOT_INIT;
    }
    if (raw_out == NULL)
    {
        return MQ135_ERR_NOT_INIT;
    }

    adc_select_input(MQ135_ADC_CHANNEL);
    *raw_out = adc_read();

    return MQ135_OK;
}

Mq135Result MQ135_Read(Mq135Data *data)
{
    uint16_t raw;
    Mq135Result res;

    if (!sInitOk)
    {
        return MQ135_ERR_NOT_INIT;
    }
    if (data == NULL)
    {
        return MQ135_ERR_NOT_INIT;
    }

    adc_select_input(MQ135_ADC_CHANNEL);
    raw = adc_read();

    data->raw_adc           = raw;
    data->adc_voltage_mv    = (float)raw * (float)ADC_VREF_MV / (float)ADC_MAX_COUNT;
    data->sensor_voltage_mv = data->adc_voltage_mv * DIVIDER_GAIN;

    return MQ135_OK;
}
