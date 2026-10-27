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
 * File:        air_quality.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       MQ135 air-quality processing module (payload layer).
 * \details     Converts raw MQ135 driver output into sensor resistance Rs,
 *              the Rs/R0 ratio, and a bounded relative air-quality response
 *              value suitable for the AIR-QUALITY vs ALTITUDE experiment.
 *
 *              THIS MODULE DOES NOT:
 *              - calculate official AQI
 *              - claim gas-specific ppm
 *              - perform altitude correlation
 *              - log or transmit data
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef AIR_QUALITY_H_
#define AIR_QUALITY_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes for the air-quality module.
 */
typedef enum
{
	AQ_OK = 0,              /**< Processing succeeded.                     */
	AQ_ERR_INVALID_INPUT,   /**< Sensor voltage is zero or negative.       */
	AQ_ERR_NOT_INIT         /**< Module used before AirQuality_Init().     */
} AqResult;

/**
 * \brief   Fully processed MQ135 air-quality data.
 */
typedef struct
{
	/* Raw hardware values (preserved for debugging / recalibration) */
	uint16_t    raw_adc;                /**< 12-bit ADC count (0–4095).               */
	float       adc_voltage_mv;         /**< Voltage at ADC pin (mV).                 */
	float       sensor_voltage_mv;      /**< MQ135 AOUT voltage before divider (mV).  */

	/* Resistance model */
	float       rs_ohms;                /**< Calculated sensor resistance Rs (Ω).     */
	float       rs_over_r0;            /**< Rs / R0  (dimensionless ratio).           */

	/**
	 * \brief   Relative air-quality response (0.0 – 100.0).
	 *
	 * \details Defined as:
	 *
	 *            response = clamp(100.0 / rs_over_r0, 0, 100)
	 *
	 *          Semantic:
	 *            HIGH value (→ 100) = LOWER Rs/R0 = stronger MQ135 pollution response.
	 *            LOW  value (→   0) = HIGHER Rs/R0 = cleaner relative air response.
	 *
	 *          This is NOT official AQI.  It is a provisional relative indicator
	 *          using an uncalibrated R0 (see config.h MQ135_R0_OHMS_PROVISIONAL).
	 *
	 * \warning Replace R0 after physical clean-air calibration before
	 *          treating this value as quantitatively meaningful.
	 */
	float       air_quality_response;

	bool        valid;                  /**< true if data was computed without error.  */
} AqData;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the air-quality processing module.
 * \details Validates that the MQ135 driver is already initialised.
 *          Does NOT re-initialise the driver.
 *
 * \return  AQ_OK           on success.
 * \return  AQ_ERR_NOT_INIT if the MQ135 driver is not yet initialised.
 */
AqResult AirQuality_Init(void);

/**
 * \brief   Checks whether the module is initialised.
 *
 * \return  true if AirQuality_Init() completed without error.
 */
bool AirQuality_IsInitialised(void);

/**
 * \brief   Reads the MQ135 driver and computes all processed values.
 *
 * \param[out] data   Pointer to receive the full processed data.
 *
 * \return  AQ_OK                 on success.
 * \return  AQ_ERR_NOT_INIT       if the module is not initialised.
 * \return  AQ_ERR_INVALID_INPUT  if sensor voltage is zero (ADC reads zero).
 */
AqResult AirQuality_Read(AqData *data);

#endif /* AIR_QUALITY_H_ */
