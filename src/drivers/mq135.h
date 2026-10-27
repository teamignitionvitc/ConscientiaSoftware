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
 * File:        mq135.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Header file for MQ135 air quality sensor driver.
 * \details     Provides raw ADC reading and voltage conversion for the MQ135
 *              analog gas sensor connected via an external voltage divider to
 *              ADC0 on the Waveshare RP2040-Zero.  This is a hardware driver
 *              only — no AQI calculation, calibration curves, or telemetry.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef MQ135_H_
#define MQ135_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by MQ135 driver functions.
 */
typedef enum
{
	MQ135_OK = 0,               /**< Operation succeeded.                   */
	MQ135_ERR_NOT_INIT          /**< Driver used before successful init.    */
} Mq135Result;

/**
 * \brief   MQ135 measurement data.
 */
typedef struct
{
	uint16_t    raw_adc;            /**< 12-bit ADC count (0–4095).                     */
	float       adc_voltage_mv;     /**< Voltage at the ADC pin (mV).                   */
	float       sensor_voltage_mv;  /**< Estimated voltage at MQ135 AOUT before
	                                     the voltage divider (mV).                       */
} Mq135Data;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the MQ135 driver and the RP2040 ADC.
 * \details Calls adc_init(), configures GPIO26 for analog input, and
 *          selects ADC channel 0 (per config.h).
 *
 * \return  MQ135_OK always (ADC init cannot fail on RP2040).
 */
Mq135Result MQ135_Init(void);

/**
 * \brief   Checks whether the driver has been initialised.
 *
 * \return  true if MQ135_Init() has been called, false otherwise.
 */
bool MQ135_IsInitialised(void);

/**
 * \brief   Reads the raw 12-bit ADC count.
 *
 * \param[out] raw_out   Pointer to receive the ADC count (0–4095).
 *
 * \return  MQ135_OK            on success.
 * \return  MQ135_ERR_NOT_INIT  if the driver is not initialised.
 */
Mq135Result MQ135_ReadRaw(uint16_t *raw_out);

/**
 * \brief   Reads the MQ135 and returns raw ADC, ADC voltage, and estimated
 *          sensor output voltage (before the divider).
 *
 * \param[out] data   Pointer to receive all measurements.
 *
 * \return  MQ135_OK            on success.
 * \return  MQ135_ERR_NOT_INIT  if the driver is not initialised.
 */
Mq135Result MQ135_Read(Mq135Data *data);

#endif /* MQ135_H_ */
