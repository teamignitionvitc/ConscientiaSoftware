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
 * File:        lora_e32.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Header file for LoRa E32-433T3D radio module driver.
 * \details     Provides transparent UART mode initialisation, send/receive,
 *              non-blocking poll, and AUX-pin ready-check APIs for the
 *              EBYTE E32-433T3D on the Waveshare RP2040-Zero.
 *              Hardware driver only — no telemetry framing or payload logic.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef LORA_E32_H_
#define LORA_E32_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * Macro Definitions
 **************************************************************************************************/

/** \brief  Maximum payload bytes per transparent-mode transmission. */
#define LORA_E32_MAX_PAYLOAD        (58U)

/** \brief  AUX-high timeout (ms) after asserting normal mode. */
#define LORA_E32_AUX_TIMEOUT_MS     (1000U)

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by LoRa E32 driver functions.
 */
typedef enum
{
	LORA_OK = 0,                /**< Operation succeeded.                    */
	LORA_ERR_NOT_INIT,          /**< Driver used before successful init.     */
	LORA_ERR_AUX_TIMEOUT,       /**< AUX pin did not go high within timeout. */
	LORA_ERR_NO_DATA,           /**< No bytes available to receive.          */
	LORA_ERR_OVERFLOW           /**< Receive buffer overflowed.              */
} LoraE32Result;

/**
 * \brief   Operating mode of the E32 module (M0 / M1 pin states).
 */
typedef enum
{
	LORA_MODE_NORMAL   = 0,     /**< M0=0, M1=0 — transparent UART.         */
	LORA_MODE_WAKEUP   = 1,     /**< M0=1, M1=0 — wake-up preamble mode.    */
	LORA_MODE_POWER_SAVE = 2,   /**< M0=0, M1=1 — power-saving mode.        */
	LORA_MODE_SLEEP    = 3      /**< M0=1, M1=1 — sleep / config mode.      */
} LoraE32Mode;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the LoRa E32 driver, UART0, and control GPIO pins.
 * \details Sets up UART0 at 9600 baud (GPIO0/1), configures M0/M1/AUX
 *          as output/output/input, places the module in NORMAL mode, and
 *          waits for AUX to assert ready.
 *
 * \return  LORA_OK               on success.
 * \return  LORA_ERR_AUX_TIMEOUT  if AUX does not go high after mode switch.
 */
LoraE32Result LoraE32_Init(void);

/**
 * \brief   Checks whether the driver has been initialised.
 */
bool LoraE32_IsInitialised(void);

/**
 * \brief   Sends a byte buffer in transparent mode.
 * \details Blocks only for the duration of the UART write (byte-by-byte).
 *          The caller is responsible for ensuring the module is not busy
 *          (check LoraE32_IsAuxReady() before calling if required).
 *
 * \param[in] data    Pointer to data bytes to transmit.
 * \param[in] len     Number of bytes (≤ LORA_E32_MAX_PAYLOAD).
 *
 * \return  LORA_OK             on success.
 * \return  LORA_ERR_NOT_INIT   if not initialised.
 */
LoraE32Result LoraE32_Send(const uint8_t *data, uint8_t len);

/**
 * \brief   Non-blocking receive poll: reads all available UART bytes.
 *
 * \param[out] buf      Buffer to receive bytes into.
 * \param[in]  buf_len  Size of the caller-supplied buffer.
 * \param[out] bytes_read  Number of bytes actually read.
 *
 * \return  LORA_OK           on success (may be 0 bytes if nothing arrived).
 * \return  LORA_ERR_NOT_INIT if not initialised.
 * \return  LORA_ERR_OVERFLOW if available bytes exceeded buf_len (buf
 *                            contains the first buf_len bytes).
 */
LoraE32Result LoraE32_Receive(uint8_t *buf, uint8_t buf_len,
                               uint8_t *bytes_read);

/**
 * \brief   Checks whether the AUX pin reports the module is ready.
 * \details AUX = HIGH means the module has finished its current operation
 *          and is ready for the next command or data.
 *
 * \return  true if AUX is high, false otherwise.
 */
bool LoraE32_IsAuxReady(void);

/**
 * \brief   Waits until AUX goes HIGH or the timeout elapses.
 *
 * \param[in] timeout_ms   Maximum wait time in milliseconds.
 *
 * \return  LORA_OK               if AUX went high within the timeout.
 * \return  LORA_ERR_AUX_TIMEOUT  if the timeout elapsed before AUX was ready.
 */
LoraE32Result LoraE32_WaitAuxReady(uint32_t timeout_ms);

/**
 * \brief   Sets the E32 operating mode via M0/M1 pins.
 * \details After changing mode the driver waits for AUX to re-assert.
 *
 * \param[in] mode   Desired operating mode.
 *
 * \return  LORA_OK               on success.
 * \return  LORA_ERR_NOT_INIT     if not initialised.
 * \return  LORA_ERR_AUX_TIMEOUT  if AUX did not assert within timeout.
 */
LoraE32Result LoraE32_SetMode(LoraE32Mode mode);

#endif /* LORA_E32_H_ */
