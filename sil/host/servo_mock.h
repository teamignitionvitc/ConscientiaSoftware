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

                    © Copyright of Ignition Avionics Department
*/

/**************************************************************************************************
 * File:        servo_mock.h
 * Author:      Vidisha Jain
 * Created On:  27/10/26
 *
 * \brief       Public interface for the host-side SIL servo mock.
 * \details     Provides reset and deployment-count inspection functions used
 *              to verify one-shot recovery deployment without servo hardware.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      27/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/
 
#ifndef SIL_SERVO_MOCK_H_
#define SIL_SERVO_MOCK_H_

#include <stdint.h>

void     SilServoMock_Reset(void);
uint32_t SilServoMock_GetDeployCount(void);

#endif /* SIL_SERVO_MOCK_H_ */