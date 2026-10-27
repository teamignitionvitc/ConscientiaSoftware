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
 * File:        servo_mock.c
 * Author:      Vidisha Jain
 * Created On:  27/10/26
 *
 * \brief       Host-side servo mock for recovery SIL testing.
 * \details     Replaces the physical MG90 servo deployment function during
 *              host execution and records the number of deployment commands.
 *              Production recovery logic is compiled unchanged.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      27/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#include "servo_mock.h"
#include "drivers/servo.h"

static uint32_t sDeployCount = 0U;

void SilServoMock_Reset(void)
{
    sDeployCount = 0U;
}

uint32_t SilServoMock_GetDeployCount(void)
{
    return sDeployCount;
}

/*
 * Host-side replacement for the hardware Servo_Deploy().
 *
 * recovery.c calls this exact symbol. During SIL we deliberately do not
 * compile the real servo.c, so the linker resolves Servo_Deploy() here.
 */
ServoResult Servo_Deploy(void)
{
    sDeployCount++;
    return SERVO_OK;
}