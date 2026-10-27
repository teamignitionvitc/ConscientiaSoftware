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
 * File:        recovery.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       IMU-based drop-detection and parachute deployment module.
 * \details     Detects the CanSat DROP/RELEASE by monitoring MPU6050
 *              accelerometer magnitude for a free-fall signature
 *              (|a| → 0 g).  Upon confirmed detection, immediately calls
 *              Servo_Deploy() to open the recovery system.
 *
 *              Design constraints:
 *                - NO altitude or descent requirement — pure IMU trigger.
 *                - NO intentional delay — deploy ASAP after confirmation.
 *                - Short configurable noise-rejection counter only
 *                  (REC_FREEFALL_CONFIRM_SAMPLES in config.h).
 *                - Total response < RECOVERY_MAX_RESPONSE_TIME_MS (3 s).
 *                - Deployment is LATCHED — once triggered, never retriggered.
 *                - Non-blocking; call Recovery_Update() every main-loop tick.
 *
 *              This module does NOT manage flight state, logging, or
 *              telemetry. It calls Servo_Deploy() directly.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef RECOVERY_H_
#define RECOVERY_H_

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
 * \brief   Result codes returned by recovery module functions.
 */
typedef enum
{
	REC_OK = 0,             /**< Operation succeeded.                         */
	REC_ERR_NOT_INIT        /**< Module used before Recovery_Init().          */
} RecResult;

/**
 * \brief   Reason the deployment was triggered.
 */
typedef enum
{
	REC_REASON_NONE = 0,    /**< Not yet deployed.                            */
	REC_REASON_FREEFALL     /**< Free-fall detected (|a| < threshold).        */
} RecReason;

/**
 * \brief   Recovery module status snapshot.
 */
typedef struct
{
	bool        deployed;           /**< true once Servo_Deploy() has fired.  */
	RecReason   reason;             /**< What triggered deployment.           */
	uint32_t    detect_time_ms;     /**< Timestamp when drop was confirmed.  */
	uint32_t    deploy_time_ms;     /**< Timestamp when servo was commanded. */
	float       accel_mag_g;        /**< Latest |a| magnitude (g).           */
	uint8_t     confirm_count;      /**< Running confirmation counter.       */
} RecStatus;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the recovery module.
 * \details Must be called after SensorManager_Init() and Servo_Init().
 *
 * \return  REC_OK always.
 */
RecResult Recovery_Init(void);

/** \brief  Returns true if the module has been initialised. */
bool Recovery_IsInitialised(void);

/**
 * \brief   Non-blocking recovery update — call every main-loop tick.
 * \details Evaluates IMU data for free-fall.  If the confirmation
 *          counter reaches REC_FREEFALL_CONFIRM_SAMPLES, calls
 *          Servo_Deploy() immediately and latches.
 *
 * \param[in] data    Latest SmData snapshot from SensorManager.
 * \param[in] now_ms  Current RP2040 millisecond timestamp.
 *
 * \return  REC_OK / REC_ERR_NOT_INIT.
 */
RecResult Recovery_Update(const SmData *data, uint32_t now_ms);

/**
 * \brief   Returns true if the recovery system has been deployed.
 */
bool Recovery_IsDeployed(void);

/**
 * \brief   Copies the current recovery status snapshot.
 *
 * \param[out] status   Pointer to receive the status copy.
 *
 * \return  REC_OK / REC_ERR_NOT_INIT.
 */
RecResult Recovery_GetStatus(RecStatus *status);

#endif /* RECOVERY_H_ */
