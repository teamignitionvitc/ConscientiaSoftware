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
 * File:        wind_inference.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Post-flight wind inference payload module header.
 * \details     Records altitude-tagged horizontal GNSS velocity components
 *              suitable for post-flight wind direction / magnitude vs
 *              altitude analysis.
 *
 *              Physical model
 *              ──────────────
 *              During DESCENT under parachute the CanSat has negligible
 *              self-propulsion.  Its horizontal motion is dominated by the
 *              ambient wind field.  Therefore:
 *
 *                  V_ground ≈ V_wind + V_cansat_aero
 *
 *              where V_cansat_aero is the small residual aerodynamic
 *              drift/pendulum velocity.  For a first-order estimate:
 *
 *                  V_wind ≈ V_ground    (during descent)
 *
 *              This is a MEASURED ground-velocity record, not a real-time
 *              solved wind vector.  The true wind field should be extracted
 *              in post-flight processing by a ground-station tool that can
 *              apply parachute aerodynamic corrections.
 *
 *              This module stores a clearly separated pair:
 *                1. MEASURED ground velocity (from GNSS).
 *                2. INFERRED wind estimate (same value for now, clearly
 *                   marked provisional).
 *
 *              The IMU is used ONLY for a motion-quality flag (high gyro
 *              rates → pendulum swing → lower confidence).  It is NOT
 *              used as an absolute heading reference.
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

#ifndef WIND_INFERENCE_H_
#define WIND_INFERENCE_H_

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
 * \brief   Result codes returned by wind inference functions.
 */
typedef enum
{
	WI_OK = 0,              /**< Operation succeeded.                         */
	WI_ERR_NOT_INIT,        /**< Module used before WindInfer_Init().         */
	WI_ERR_NO_DATA          /**< No valid GNSS/alt sample available.         */
} WiResult;

/**
 * \brief   Motion-quality tier derived from IMU gyro rates.
 */
typedef enum
{
	WI_QUALITY_UNKNOWN = 0, /**< No IMU data available.                      */
	WI_QUALITY_GOOD,        /**< Low gyro rates — stable descent.            */
	WI_QUALITY_FAIR,        /**< Moderate gyro rates — some pendulum motion. */
	WI_QUALITY_POOR         /**< High gyro rates — aggressive swinging.      */
} WiQuality;

/**
 * \brief   Altitude-tagged horizontal velocity and wind-estimate snapshot.
 */
typedef struct
{
	/* ── Measured ground velocity (GNSS) ───────────────────────────── */
	float       gnd_speed_mps;      /**< GNSS ground speed (m/s).            */
	float       gnd_course_deg;     /**< True course / track (°, 0 = N).     */
	float       gnd_vn_mps;         /**< Northward component (m/s).          */
	float       gnd_ve_mps;         /**< Eastward  component (m/s).          */

	/* ── Inferred wind estimate (PROVISIONAL) ──────────────────────── */
	float       wind_speed_mps;     /**< Inferred wind speed (m/s).
	                                     Currently = gnd_speed_mps.          */
	float       wind_dir_deg;       /**< Inferred wind FROM direction (°).
	                                     Currently = gnd_course_deg + 180.   */
	float       wind_vn_mps;        /**< Inferred northward wind (m/s).      */
	float       wind_ve_mps;        /**< Inferred eastward  wind (m/s).      */

	/* ── Altitude ──────────────────────────────────────────────────── */
	float       altitude_agl_m;     /**< Baro altitude AGL (m).              */
	float       altitude_msl_m;     /**< Baro altitude MSL (m).              */

	/* ── Quality / status ──────────────────────────────────────────── */
	WiQuality   quality;            /**< IMU-derived motion quality tier.     */
	float       gyro_mag_dps;       /**< |gyro| magnitude (°/s) at sample.   */
	uint8_t     satellites;         /**< GNSS satellites in use.             */

	/* ── Validity ──────────────────────────────────────────────────── */
	bool        gnss_valid;         /**< GNSS fix was present and usable.    */
	bool        alt_valid;          /**< Baro altitude was present.          */
	bool        sample_valid;       /**< Both GNSS and alt valid this tick.  */

	/* ── Timestamp ─────────────────────────────────────────────────── */
	uint32_t    timestamp_ms;       /**< RP2040 ms timestamp of this sample. */
} WiData;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the wind inference module.
 *
 * \return  WI_OK always.
 */
WiResult WindInfer_Init(void);

/** \brief  Returns true if the module has been initialised. */
bool WindInfer_IsInitialised(void);

/**
 * \brief   Processes the latest sensor snapshot and updates the internal
 *          wind-inference record.
 *
 * \param[in] data    Latest SmData snapshot from SensorManager.
 * \param[in] alt_agl Barometric altitude AGL (m) from FlightState.
 * \param[in] now_ms  Current RP2040 millisecond timestamp.
 *
 * \return  WI_OK / WI_ERR_NOT_INIT.
 */
WiResult WindInfer_Update(const SmData *data, float alt_agl, uint32_t now_ms);

/**
 * \brief   Retrieves the latest wind-inference snapshot.
 *
 * \param[out] out   Pointer to receive the data copy.
 *
 * \return  WI_OK / WI_ERR_NOT_INIT / WI_ERR_NO_DATA.
 */
WiResult WindInfer_GetData(WiData *out);

#endif /* WIND_INFERENCE_H_ */
