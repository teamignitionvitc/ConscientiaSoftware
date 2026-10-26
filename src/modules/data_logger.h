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
 * \brief       1 CSV = 1 graph data logger for GLANCE post-flight analysis.
 * \details     Each CSV contains exactly the X/Y data for one graph:
 *
 *                altitude_time.csv   — altitude vs time
 *                pressure_alt.csv    — pressure vs altitude
 *                temp_alt.csv        — temperature vs altitude
 *                accel_alt.csv       — accel X/Y/Z vs altitude
 *                gyro_alt.csv        — gyro X/Y/Z vs altitude
 *                pm_alt.csv          — PM1.0/2.5/10 vs altitude
 *                airqual_alt.csv     — AQ response vs altitude
 *                light_alt.csv       — fused lux vs altitude
 *                windspd_alt.csv     — wind speed vs altitude
 *                winddir_alt.csv     — wind direction vs altitude
 *                gnss_track.csv      — lat/lon/alt trajectory
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 * 001  MOD      26/10/26   Vidisha Jain         Rewritten: 1 CSV = 1 graph
 **************************************************************************************************/

#ifndef DATA_LOGGER_H_
#define DATA_LOGGER_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

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
	DL_OK = 0,
	DL_ERR_NOT_INIT,
	DL_ERR_SD,
	DL_ERR_FS,
	DL_ERR_FILE
} DlResult;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

DlResult DataLogger_Init(void);
bool     DataLogger_IsInitialised(void);
DlResult DataLogger_Open(void);
DlResult DataLogger_Sync(uint32_t now_ms);
DlResult DataLogger_Close(void);

/* 1 CSV = 1 graph row writers */
DlResult DataLogger_LogAltTime(uint32_t ts_ms, float alt_agl_m);
DlResult DataLogger_LogPressure(float alt_agl_m, uint32_t pressure_pa);
DlResult DataLogger_LogTemp(float alt_agl_m, int32_t temp_cdegc);
DlResult DataLogger_LogAccelX(float alt_agl_m, float ax);
DlResult DataLogger_LogAccelY(float alt_agl_m, float ay);
DlResult DataLogger_LogAccelZ(float alt_agl_m, float az);
DlResult DataLogger_LogGyroX(float alt_agl_m, float gx);
DlResult DataLogger_LogGyroY(float alt_agl_m, float gy);
DlResult DataLogger_LogGyroZ(float alt_agl_m, float gz);
DlResult DataLogger_LogPM1(float alt_agl_m, uint16_t pm1);
DlResult DataLogger_LogPM25(float alt_agl_m, uint16_t pm25);
DlResult DataLogger_LogPM10(float alt_agl_m, uint16_t pm10);
DlResult DataLogger_LogAirQual(float alt_agl_m, float response);
DlResult DataLogger_LogLight(float alt_agl_m, float fused_lux);
DlResult DataLogger_LogWindSpd(float alt_agl_m, float wind_speed_mps);
DlResult DataLogger_LogWindDir(float alt_agl_m, float wind_dir_deg);
DlResult DataLogger_LogGnss(uint32_t ts_ms, double lat, double lon,
                            float alt_m, float speed, float course, uint8_t sats);

#endif /* DATA_LOGGER_H_ */
