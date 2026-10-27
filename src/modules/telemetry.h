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
 * File:        telemetry.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       LoRa downlink telemetry module header.
 * \details     Formats SmData sensor snapshots into a compact ASCII
 *              telemetry packet and transmits it over the LoRa E32 radio
 *              at the rate defined by TELEMETRY_TX_INTERVAL_MS in config.h.
 *
 *              Packet format is provisional CSV-on-air designed for easy
 *              ground-station parsing.  The final frame schema must be
 *              agreed with the GS software team before the launch flight.
 *
 *              This module does NOT perform payload processing, flight-state
 *              decisions, recovery logic, SD logging or sensor management.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 * 001  MOD      27/10/26   Vidisha Jain         Added supplemental telemetry values
 **************************************************************************************************/

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "modules/sensor_manager.h"

/**************************************************************************************************
 * Macro Definitions
 **************************************************************************************************/

/** \brief  Maximum length of one formatted telemetry packet (bytes). */
#define TELEMETRY_PKT_MAX_LEN       (192U)

/** \brief  Synchronisation preamble prepended to every packet. */
#define TELEMETRY_SYNC              "$$IGNI,"

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by telemetry module functions.
 */
typedef enum
{
	TM_OK = 0,              /**< Operation succeeded.                         */
	TM_ERR_NOT_INIT,        /**< Module used before Telemetry_Init().         */
	TM_ERR_LORA,            /**< LoRa driver not ready / send failed.         */
	TM_ERR_FORMAT           /**< Packet formatting error (buffer overflow).   */
} TmResult;
/**
 * \brief   Supplemental processed/system values included in live telemetry.
 */
typedef struct
{
    float   fused_lux;
    float   air_quality_response;

    uint8_t flight_state;
    bool    recovery_deployed;

    bool    fused_lux_valid;
    bool    air_quality_valid;
} TmSupplemental;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the telemetry module and the LoRa E32 driver.
 * \details Calls LoraE32_Init() and places the module in NORMAL mode.
 *          After this call LoraE32_IsInitialised() will be true on success.
 *
 * \return  TM_OK           on success.
 * \return  TM_ERR_LORA     if LoRa driver initialisation fails.
 */
TmResult Telemetry_Init(void);

/** \brief  Returns true if Telemetry_Init() completed without error. */
bool Telemetry_IsInitialised(void);

/**
 * \brief   Periodic transmit update — call from main loop every iteration.
 * \details Transmits one formatted packet if TELEMETRY_TX_INTERVAL_MS has
 *          elapsed since the last transmission.  Non-blocking; checks AUX
 *          ready before sending and skips if module is busy.
 *
 * \param[in] data    Latest sensor data snapshot to transmit.
 * \param[in] now_ms  Current millisecond timestamp.
 *
 * \return  TM_OK / TM_ERR_NOT_INIT / TM_ERR_LORA / TM_ERR_FORMAT.
 */
TmResult Telemetry_Update(const SmData *data,
                          const TmSupplemental *supp,
                          uint32_t now_ms);
/**
 * \brief   Formats a telemetry packet from sensor data without transmitting.
 * \details Useful for testing / ground-station format validation.
 *
 * \param[in]  data    Sensor data to encode.
 * \param[out] buf     Caller buffer to receive the formatted packet.
 * \param[in]  buf_len Size of the caller buffer (≥ TELEMETRY_PKT_MAX_LEN).
 * \param[out] pkt_len Actual number of bytes written (excl. null terminator).
 *
 * \return  TM_OK / TM_ERR_FORMAT.
 */
TmResult Telemetry_FormatPacket(const SmData *data,
                                const TmSupplemental *supp,
                                char *buf, uint8_t buf_len,
                                uint8_t *pkt_len);
#endif /* TELEMETRY_H_ */
