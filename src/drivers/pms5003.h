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
 * File:        pms5003.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Header file for PMS5003 particulate matter sensor driver.
 * \details     Provides continuous frame reception and parsing over PIO UART
 *              for the Plantower PMS5003 on the Waveshare RP2040-Zero.
 *              This is a hardware driver only — no AQI calculation, altitude
 *              correlation, or telemetry logic.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef PMS5003_H_
#define PMS5003_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * Macro Definitions
 **************************************************************************************************/

/** \brief  PMS5003 frame start bytes. */
#define PMS5003_START_BYTE_1            (0x42U)
#define PMS5003_START_BYTE_2            (0x4DU)

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by PMS5003 driver functions.
 */
typedef enum
{
	PMS5003_OK = 0,             /**< Operation / poll succeeded.            */
	PMS5003_ERR_I2C,            /**< PIO UART communication failure.        */
	PMS5003_ERR_NOT_INIT,       /**< Driver used before successful init.    */
	PMS5003_ERR_NO_DATA,        /**< No valid frame available yet.          */
	PMS5003_ERR_CHECKSUM,       /**< Last frame failed checksum.            */
	PMS5003_ERR_FRAME           /**< Frame length / header mismatch.        */
} Pms5003Result;

/**
 * \brief   Parsed PMS5003 measurement data.
 */
typedef struct
{
	/* Standard particle concentrations (CF=1, factory environment) µg/m³ */
	uint16_t    pm1_0_std;
	uint16_t    pm2_5_std;
	uint16_t    pm10_std;

	/* Atmospheric environment concentrations µg/m³ — PRIMARY VALUES */
	uint16_t    pm1_0_atm;
	uint16_t    pm2_5_atm;
	uint16_t    pm10_atm;

	/* Particle counts per 0.1 L of air */
	uint16_t    cnt_0_3um;
	uint16_t    cnt_0_5um;
	uint16_t    cnt_1_0um;
	uint16_t    cnt_2_5um;
	uint16_t    cnt_5_0um;
	uint16_t    cnt_10um;
} Pms5003Data;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the PMS5003 driver and PIO UART RX.
 * \details Loads the PIO UART RX program, configures GPIO9 as RX at 9600
 *          baud using PIO0 SM1 (per config.h).  The PMS5003 operates in its
 *          default active mode, streaming frames at ~1 Hz.
 *
 * \return  PMS5003_OK          on success.
 * \return  PMS5003_ERR_I2C     if PIO program cannot be loaded.
 */
Pms5003Result PMS5003_Init(void);

/**
 * \brief   Checks whether the driver has been initialised successfully.
 *
 * \return  true if PMS5003_Init() completed without error, false otherwise.
 */
bool PMS5003_IsInitialised(void);

/**
 * \brief   Non-blocking poll: drains available PIO UART bytes and parses frames.
 * \details Must be called frequently (at least every ~15 ms) to avoid FIFO
 *          overflow.  Internally synchronises on the 0x42 0x4D header,
 *          validates frame length and checksum, and stores the latest valid
 *          reading.
 *
 * \return  PMS5003_OK            if a new valid frame was completed this call.
 * \return  PMS5003_ERR_NOT_INIT  if the driver is not initialised.
 * \return  PMS5003_ERR_NO_DATA   if no complete frame was assembled yet.
 * \return  PMS5003_ERR_CHECKSUM  if a completed frame had a bad checksum.
 * \return  PMS5003_ERR_FRAME     if frame length field was invalid.
 */
Pms5003Result PMS5003_Poll(void);

/**
 * \brief   Retrieves the most recent valid PMS5003 measurement.
 *
 * \param[out] data   Pointer to receive the parsed data.
 *
 * \return  PMS5003_OK            on success.
 * \return  PMS5003_ERR_NOT_INIT  if not initialised.
 * \return  PMS5003_ERR_NO_DATA   if no valid frame has been received yet.
 */
Pms5003Result PMS5003_GetData(Pms5003Data *data);

#endif /* PMS5003_H_ */
