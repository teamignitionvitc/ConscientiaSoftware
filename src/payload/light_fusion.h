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
 * File:        light_fusion.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Multi-sensor light fusion payload module header.
 * \details     Fuses the three BH1750 lux readings into a single best-
 *              estimate lux value, paired with barometric altitude and
 *              sensor-quality metadata.
 *
 *              Raw per-sensor lux and validity flags are preserved alongside
 *              the fused output.  Sensor disagreement (max − min spread) is
 *              computed to flag possible occlusion or malfunction.
 *
 *              The fusion method is PROVISIONAL — currently a simple
 *              equal-weight mean of valid sensors.  Future work should
 *              replace this with a weighted or outlier-rejecting scheme
 *              once flight data is available for characterisation.
 *
 *              Flight-state awareness is accepted as an input to let the
 *              caller indicate motion quality, but the module does NOT
 *              import the flight-state module header itself to avoid
 *              unnecessary coupling.
 *
 *              This module does NOT log, transmit, or display data.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef LIGHT_FUSION_H_
#define LIGHT_FUSION_H_

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
 * \brief   Result codes returned by light fusion functions.
 */
typedef enum
{
	LF_OK = 0,              /**< Operation succeeded.                         */
	LF_ERR_NOT_INIT,        /**< Module used before LightFusion_Init().      */
	LF_ERR_NO_DATA          /**< No valid light data available.              */
} LfResult;

/**
 * \brief   Motion quality hint passed by the caller.
 * \details Decoupled from flight_state.h — the caller maps FlightPhase to
 *          this enum so that light_fusion does not depend on the state
 *          machine module directly.
 */
typedef enum
{
	LF_MOTION_UNKNOWN = 0,  /**< No information (default).                   */
	LF_MOTION_STATIC,       /**< Sensor array is stationary (pre-launch/landed). */
	LF_MOTION_DYNAMIC       /**< Sensor array is moving (ascent/descent).    */
} LfMotionHint;

/**
 * \brief   Fused light measurement snapshot.
 */
typedef struct
{
	/* Fused output */
	float       fused_lux;          /**< Best-estimate illuminance (lux).     */
	float       altitude_m;         /**< BMP280 baro altitude AGL (m).        */

	/* Per-sensor raw values (preserved) */
	float       lux1;               /**< BH1750 #1 raw reading (lux).         */
	float       lux2;               /**< BH1750 #2 raw reading (lux).         */
	float       lux3;               /**< BH1750 #3 raw reading (lux).         */

	/* Per-sensor validity */
	bool        valid1;             /**< BH1750 #1 reading is valid.          */
	bool        valid2;             /**< BH1750 #2 reading is valid.          */
	bool        valid3;             /**< BH1750 #3 reading is valid.          */

	/* Quality metrics */
	float       spread_lux;         /**< Sensor disagreement: max − min (lux).*/
	uint8_t     sensors_used;       /**< Number of valid sensors in fusion.   */

	/* Motion context */
	LfMotionHint motion;            /**< Caller-supplied motion quality hint. */

	/* Timestamps */
	uint32_t    light_timestamp_ms; /**< Time of light readings (ms).         */
	uint32_t    baro_timestamp_ms;  /**< Time of altitude reading (ms).       */

	/* Aggregate validity */
	bool        fused_valid;        /**< At least one sensor contributed.     */
	bool        alt_valid;          /**< Altitude data is valid.              */
} LfData;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the light fusion module.
 *
 * \return  LF_OK always.
 */
LfResult LightFusion_Init(void);

/** \brief  Returns true if the module has been initialised. */
bool LightFusion_IsInitialised(void);

/**
 * \brief   Processes the latest sensor snapshot and updates the fused light
 *          output.
 *
 * \param[in] data    Latest SmData snapshot from SensorManager.
 * \param[in] alt_agl Barometric altitude AGL (m) from FlightState.
 * \param[in] motion  Caller's hint about current motion quality.
 *
 * \return  LF_OK / LF_ERR_NOT_INIT.
 */
LfResult LightFusion_Update(const SmData *data, float alt_agl,
                            LfMotionHint motion);

/**
 * \brief   Retrieves the latest fused light measurement snapshot.
 *
 * \param[out] out   Pointer to receive the data copy.
 *
 * \return  LF_OK / LF_ERR_NOT_INIT / LF_ERR_NO_DATA.
 */
LfResult LightFusion_GetData(LfData *out);

#endif /* LIGHT_FUSION_H_ */
