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
                         .::::          ::          ::::
                       .::::::::.       ::       .:::::::::
                       ::::::::::::....::::.....:::::::::::
                        .:::::::::::::::::::::::::::::::::.

                    © Copyright of Ignition Software Department
*/

/**************************************************************************************************
 * File:        servo.h
 * Author:      Vidisha Jain
 * Created On:  27/10/26
 *
 * \brief       Minimal host-only servo interface stub for SIL compilation.
 * \details     Provides the production-compatible ServoResult and
 *              Servo_Deploy declaration required when recovery.c is compiled
 *              on the host without RP2040 PWM hardware.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      27/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/
#ifndef SERVO_H_
#define SERVO_H_

typedef enum
{
    SERVO_OK = 0,
    SERVO_ERR_NOT_INIT
} ServoResult;

ServoResult Servo_Deploy(void);

#endif /* SERVO_H_ */