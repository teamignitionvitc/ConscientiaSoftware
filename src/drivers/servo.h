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
                        .:::::::::::::::::::::::::::::::::.        

                    © Copyright of Ignition Software Department
*/

/**************************************************************************************************
 * File:        servo.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Header file for MG90S servo PWM driver.
 * \details     Controls the MG90S servo on GPIO7 via RP2040 hardware PWM
 *              at 50 Hz using pulse width limits from config.h.
 *              Hardware driver only — no recovery logic or state machine.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef SERVO_H_
#define SERVO_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by servo driver functions.
 */
typedef enum
{
	SERVO_OK = 0,               /**< Operation succeeded.                    */
	SERVO_ERR_NOT_INIT          /**< Driver used before successful init.     */
} ServoResult;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises GPIO7 PWM for 50 Hz servo control.
 * \details Configures PWM slice with sys_clk / 125 divider and 20000-count
 *          wrap, giving a 1 µs / count resolution.  Servo is placed at the
 *          neutral pulse width (SERVO_PULSE_NEUTRAL_US) on init.
 *
 * \return  SERVO_OK always (PWM init cannot fail on RP2040).
 */
ServoResult Servo_Init(void);

/** \brief  Returns true if Servo_Init() has been called. */
bool Servo_IsInitialised(void);

/**
 * \brief   Sets the servo pulse width in microseconds.
 * \details Input is clamped to [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US]
 *          before applying.  No blocking — PWM hardware continues cycling.
 *
 * \param[in] pulse_us   Desired pulse width in microseconds.
 *
 * \return  SERVO_OK / SERVO_ERR_NOT_INIT.
 */
ServoResult Servo_SetPulse(uint16_t pulse_us);

/**
 * \brief   Sets the servo position by angle (0–180 degrees).
 * \details Linearly maps the angle to the configured [MIN, MAX] pulse range.
 *          Input clamped to [0, 180].
 *
 * \param[in] angle_deg   Target angle in degrees.
 *
 * \return  SERVO_OK / SERVO_ERR_NOT_INIT.
 */
ServoResult Servo_SetAngle(uint8_t angle_deg);

/**
 * \brief   Commands the servo to the stowed position (minimum pulse).
 * \details Moves to SERVO_PULSE_MIN_US — physically the stowed/retracted
 *          position for the recovery mechanism.
 *
 * \return  SERVO_OK / SERVO_ERR_NOT_INIT.
 */
ServoResult Servo_Stow(void);

/**
 * \brief   Commands the servo to the deployed position (maximum pulse).
 * \details Moves to SERVO_PULSE_MAX_US — physically the deployed/released
 *          position for the recovery mechanism.
 *
 * \return  SERVO_OK / SERVO_ERR_NOT_INIT.
 */
ServoResult Servo_Deploy(void);

/**
 * \brief   Commands the servo to the neutral (centre) position.
 *
 * \return  SERVO_OK / SERVO_ERR_NOT_INIT.
 */
ServoResult Servo_Neutral(void);

#endif /* SERVO_H_ */
