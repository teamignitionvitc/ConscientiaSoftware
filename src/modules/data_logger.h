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
 * File:        data_logger.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Multi-file FAT32 CSV data logger.
 * \details     Manages 6 laptop-readable CSV files on a FAT32-formatted
 *              micro SD card via FatFs:
 *
 *                mission.csv       — summary: baro + GNSS + flight phase
 *                imu.csv           — raw accelerometer + gyro
 *                particulate.csv   — PM vs altitude
 *                air_quality.csv   — MQ135 Rs/R0 + response
 *                light.csv         — fused + raw lux
 *                wind.csv          — GNSS velocity + inferred wind
 *
 *              Every CSV row includes timestamp_ms and alt_agl_m so
 *              datasets can be correlated in post-flight GLANCE analysis.
 *
 *              This module does NOT decide WHEN to log — the caller
 *              calls the appropriate LogXxx() function at the rate it
 *              chooses.  Periodic Sync() flushes all open files.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 * 001  MOD      26/10/26   Vidisha Jain         Rewritten for FatFs multi-file CSV
 **************************************************************************************************/

#ifndef DATA_LOGGER_H_
#define DATA_LOGGER_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/* Payload / module data types used by the log functions */
#include "modules/sensor_manager.h"
#include "payload/air_quality.h"
#include "payload/particulate_analysis.h"
#include "payload/light_fusion.h"
#include "payload/wind_inference.h"

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

typedef enum
{
	DL_OK = 0,              /**< Operation succeeded.                         */
	DL_ERR_NOT_INIT,        /**< Module not initialised.                     */
	DL_ERR_SD,              /**< SD card hardware failure.                   */
	DL_ERR_FS,              /**< FatFs mount / filesystem error.             */
	DL_ERR_FILE             /**< File open / write error.                    */
} DlResult;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises SD card hardware and mounts the FAT32 filesystem.
 * \details Calls SdCard_Init() (via FatFs diskio) then f_mount().
 *          The SD card MUST be pre-formatted FAT32 on a computer.
 */
DlResult DataLogger_Init(void);

/** \brief  Returns true if the filesystem is mounted and ready. */
bool DataLogger_IsInitialised(void);

/**
 * \brief   Creates / opens all 6 CSV files and writes their header rows.
 * \details Files are opened with FA_WRITE | FA_OPEN_APPEND so they
 *          survive a power-cycle (data is appended, not overwritten).
 */
DlResult DataLogger_Open(void);

/**
 * \brief   Syncs all open files to disk if the flush interval has elapsed.
 * \param[in] now_ms  Current RP2040 millisecond timestamp.
 */
DlResult DataLogger_Sync(uint32_t now_ms);

/**
 * \brief   Closes all open files and unmounts the filesystem.
 */
DlResult DataLogger_Close(void);

/* ── Per-dataset CSV row writers ─────────────────────────────────────── */

DlResult DataLogger_LogMission(const SmData *sm, float alt_agl, uint8_t phase,
                               uint32_t ts_ms);

DlResult DataLogger_LogImu(const SmData *sm, float alt_agl, uint32_t ts_ms);

DlResult DataLogger_LogParticulate(const PaData *pa);

DlResult DataLogger_LogAirQuality(const AqData *aq, float alt_agl,
                                  uint32_t ts_ms);

DlResult DataLogger_LogLight(const LfData *lf);

DlResult DataLogger_LogWind(const WiData *wi);

#endif /* DATA_LOGGER_H_ */
