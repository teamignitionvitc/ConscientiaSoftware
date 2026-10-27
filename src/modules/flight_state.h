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
 * File:        flight_state.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Flight-phase state machine header.
 * \details     Tracks the CanSat mission phase (PRE_LAUNCH → ASCENT → DESCENT
 *              → LANDED) using barometric altitude as the primary vertical-
 *              motion source and MPU6050 acceleration as supporting evidence.
 *
 *              State transitions are guarded by confirmation counters to reject
 *              single-sample excursions from sensor noise.  All detection
 *              thresholds are centralised in config.h.
 *
 *              This module does NOT trigger the servo, implement recovery
 *              deployment, log data, or transmit telemetry.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef FLIGHT_STATE_H_
#define FLIGHT_STATE_H_

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
 * \brief   Flight-phase enumeration.
 */
typedef enum
{
	FS_PRE_LAUNCH = 0,      /**< On the pad / in the launch vehicle.        */
	FS_ASCENT,              /**< Rising — altitude increasing.              */
	FS_DESCENT,             /**< Falling — past apogee, altitude decreasing.*/
	FS_LANDED               /**< On the ground — motion settled.            */
} FlightPhase;

/**
 * \brief   Result codes returned by flight-state functions.
 */
typedef enum
{
	FS_OK = 0,              /**< Operation succeeded.                       */
	FS_ERR_NOT_INIT         /**< Module used before FlightState_Init().     */
} FsResult;

/**
 * \brief   Observable state-machine metrics (read-only snapshot).
 */
typedef struct
{
	FlightPhase phase;              /**< Current flight phase.                */
	float       launch_alt_m;       /**< Launch-pad reference altitude (m).   */
	float       alt_agl_m;          /**< Current smoothed altitude AGL (m).   */
	float       apogee_m;           /**< Highest recorded altitude AGL (m).   */
	float       vert_rate_mps;      /**< Estimated vertical rate (m/s,
	                                     positive = upward).                  */
	float       accel_mag_g;        /**< Latest |a| magnitude (g).           */
	uint32_t    ascent_time_ms;     /**< Timestamp when ASCENT was entered.  */
	uint32_t    descent_time_ms;    /**< Timestamp when DESCENT was entered. */
	uint32_t    landed_time_ms;     /**< Timestamp when LANDED was entered.  */
	uint32_t    state_age_ms;       /**< Time spent in current state (ms).   */
} FsMetrics;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the flight-state machine.
 * \details Must be called after SensorManager_Init() has been executed.
 *          Records the first available barometric altitude as the
 *          launch-pad reference.
 *
 * \return  FS_OK always.
 */
FsResult FlightState_Init(void);

/**
 * \brief   Checks whether the module has been initialised.
 */
bool FlightState_IsInitialised(void);

/**
 * \brief   Non-blocking state-machine update.
 * \details Must be called from the main loop each tick, passing the
 *          latest sensor snapshot and current millisecond timestamp.
 *          The module evaluates transition conditions and advances the
 *          state if all confirmation guards are satisfied.
 *
 * \param[in] data    Latest sensor data snapshot from SensorManager.
 * \param[in] now_ms  Current RP2040 millisecond timestamp.
 *
 * \return  FS_OK / FS_ERR_NOT_INIT.
 */
FsResult FlightState_Update(const SmData *data, uint32_t now_ms);

/**
 * \brief   Returns the current flight phase.
 */
FlightPhase FlightState_GetPhase(void);

/**
 * \brief   Copies the full state metrics snapshot to the caller.
 *
 * \param[out] metrics   Pointer to receive the metrics copy.
 *
 * \return  FS_OK / FS_ERR_NOT_INIT.
 */
FsResult FlightState_GetMetrics(FsMetrics *metrics);

#endif /* FLIGHT_STATE_H_ */
