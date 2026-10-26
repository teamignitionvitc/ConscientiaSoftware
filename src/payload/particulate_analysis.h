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
 * File:        particulate_analysis.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       PM-vs-altitude payload analysis module header.
 * \details     Pairs PMS5003 atmospheric PM concentrations with BMP280
 *              barometric altitude to produce altitude-tagged PM data
 *              suitable for 3 separate PM-vs-altitude graphs:
 *                  PM1.0 vs altitude
 *                  PM2.5 vs altitude
 *                  PM10  vs altitude
 *
 *              No AQI conversion is performed.  This module does NOT log,
 *              transmit, or display the data — it produces the paired
 *              output structure for other modules to consume.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef PARTICULATE_ANALYSIS_H_
#define PARTICULATE_ANALYSIS_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "modules/sensor_manager.h"

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by particulate analysis functions.
 */
typedef enum
{
	PA_OK = 0,              /**< Operation succeeded.                         */
	PA_ERR_NOT_INIT,        /**< Module used before PartAnalysis_Init().      */
	PA_ERR_NO_DATA          /**< No valid PM or altitude data available.      */
} PaResult;

/**
 * \brief   Altitude-tagged PM measurement snapshot.
 */
typedef struct
{
	/* Atmospheric-environment PM concentrations (µg/m³) */
	uint16_t    pm1_0_atm;          /**< PM1.0 atmospheric (µg/m³).           */
	uint16_t    pm2_5_atm;          /**< PM2.5 atmospheric (µg/m³).           */
	uint16_t    pm10_atm;           /**< PM10  atmospheric (µg/m³).           */

	/* Barometric altitude at the time of this snapshot */
	float       altitude_m;         /**< BMP280 baro altitude AGL (m).        */
	float       altitude_msl_m;     /**< BMP280 baro altitude MSL (m).        */

	/* Timestamps */
	uint32_t    pm_timestamp_ms;    /**< Time of PM reading.                  */
	uint32_t    baro_timestamp_ms;  /**< Time of altitude reading.            */

	/* Validity */
	bool        pm_valid;           /**< PMS5003 data was fresh and valid.     */
	bool        alt_valid;          /**< BMP280 data was fresh and valid.      */
	bool        paired_valid;       /**< Both PM and altitude are valid.       */
} PaData;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the particulate analysis module.
 *
 * \return  PA_OK always.
 */
PaResult PartAnalysis_Init(void);

/** \brief  Returns true if the module has been initialised. */
bool PartAnalysis_IsInitialised(void);

/**
 * \brief   Processes the latest sensor snapshot and updates the internal
 *          altitude-tagged PM record.
 *
 * \param[in] data    Latest SmData snapshot from SensorManager.
 * \param[in] alt_agl Barometric altitude AGL from FlightState (m).
 *
 * \return  PA_OK / PA_ERR_NOT_INIT.
 */
PaResult PartAnalysis_Update(const SmData *data, float alt_agl);

/**
 * \brief   Retrieves the latest altitude-tagged PM snapshot.
 *
 * \param[out] out   Pointer to receive the data copy.
 *
 * \return  PA_OK / PA_ERR_NOT_INIT / PA_ERR_NO_DATA.
 */
PaResult PartAnalysis_GetData(PaData *out);

#endif /* PARTICULATE_ANALYSIS_H_ */
