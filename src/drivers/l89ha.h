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
 * File:        l89ha.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Header file for L89HA GNSS module driver.
 * \details     Provides non-blocking NMEA sentence reception and parsing over
 *              UART1 for the Quectel L89HA on the Waveshare RP2040-Zero.
 *              Hardware driver only — no flight/payload/telemetry logic.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef L89HA_H_
#define L89HA_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * Macro Definitions
 **************************************************************************************************/

/** \brief  Maximum length of one NMEA sentence (including \r\n). */
#define L89HA_NMEA_MAX_LEN      (82U)

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by L89HA driver functions.
 */
typedef enum
{
	L89HA_OK = 0,               /**< Operation succeeded.                    */
	L89HA_ERR_NOT_INIT,         /**< Driver used before successful init.     */
	L89HA_ERR_NO_FIX,           /**< No valid GNSS fix available.            */
	L89HA_ERR_NO_DATA,          /**< No complete sentence received yet.      */
	L89HA_ERR_CHECKSUM,         /**< Sentence failed NMEA checksum.          */
	L89HA_ERR_PARSE             /**< Sentence could not be parsed.           */
} L89haResult;

/**
 * \brief   UTC time from GNSS.
 */
typedef struct
{
	uint8_t     hours;
	uint8_t     minutes;
	uint8_t     seconds;
} L89haTime;

/**
 * \brief   Latest valid GNSS fix data.
 */
typedef struct
{
	double      latitude_deg;       /**< Decimal degrees, positive = North.      */
	double      longitude_deg;      /**< Decimal degrees, positive = East.       */
	float       altitude_m;         /**< Altitude above MSL in metres.           */
	float       speed_mps;          /**< Ground speed in m/s.                    */
	float       course_deg;         /**< True course (track) in degrees.         */
	uint8_t     satellites;         /**< Number of satellites in use.            */
	L89haTime   utc_time;           /**< UTC time of fix.                        */
	bool        fix_valid;          /**< true if the last fix was valid.         */
} L89haFix;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the L89HA driver and UART1.
 * \details Configures UART1 at 9600 baud, sets GPIO4/5 for UART function.
 *
 * \return  L89HA_OK always (UART init cannot fail at this level).
 */
L89haResult L89HA_Init(void);

/**
 * \brief   Checks whether the driver has been initialised.
 */
bool L89HA_IsInitialised(void);

/**
 * \brief   Non-blocking poll: reads available UART bytes and parses sentences.
 * \details Call frequently (every main-loop tick).  Internally assembles
 *          complete '$'…'\\n' sentences, validates the NMEA checksum, and
 *          updates the stored fix on success.
 *
 * \return  L89HA_OK          if a new valid sentence was processed.
 * \return  L89HA_ERR_NO_DATA if no complete sentence was assembled this call.
 * \return  L89HA_ERR_CHECKSUM / L89HA_ERR_PARSE on invalid data.
 */
L89haResult L89HA_Poll(void);

/**
 * \brief   Returns a copy of the latest valid GNSS fix.
 *
 * \param[out] fix   Pointer to receive fix data.
 *
 * \return  L89HA_OK          on success.
 * \return  L89HA_ERR_NO_FIX  if no valid fix has been received yet.
 */
L89haResult L89HA_GetFix(L89haFix *fix);

/**
 * \brief   Checks whether a valid GNSS fix is currently available.
 */
bool L89HA_HasFix(void);

#endif /* L89HA_H_ */
