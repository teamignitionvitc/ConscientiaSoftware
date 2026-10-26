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
 * File:        data_logger.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   1 CSV = 1 graph logger for GLANCE.  16 graph CSVs plus GNSS track data.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 * 001  MOD      26/10/26   Vidisha Jain         1 CSV = 1 graph rewrite
 **************************************************************************************************/

#include "data_logger.h"
#include "config.h"
#include "ff.h"

#include <stdio.h>
#include <string.h>

/**************************************************************************************************
 * Internal
 **************************************************************************************************/
#define DL_LINE_BUF    (160U)
#define DL_NFILES      (17U)

typedef struct { const char *name; const char *hdr; FIL fil; bool open; } DlFile;

enum {
    F_ALT_TIME = 0, F_PRESSURE, F_TEMP,
    F_ACCEL_X, F_ACCEL_Y, F_ACCEL_Z,
    F_GYRO_X, F_GYRO_Y, F_GYRO_Z,
    F_PM1, F_PM25, F_PM10,
    F_AIRQUAL, F_LIGHT, F_WINDSPD, F_WINDDIR, F_GNSS
};

static DlFile sF[DL_NFILES] = {
    { "altitude_time.csv",  "timestamp_ms,alt_agl_m\r\n", {0},false },
    { "pressure_alt.csv",   "alt_agl_m,pressure_pa\r\n", {0},false },
    { "temp_alt.csv",       "alt_agl_m,temp_cdegc\r\n", {0},false },
    { "accel_x_alt.csv",    "alt_agl_m,accel_x_g\r\n", {0},false },
    { "accel_y_alt.csv",    "alt_agl_m,accel_y_g\r\n", {0},false },
    { "accel_z_alt.csv",    "alt_agl_m,accel_z_g\r\n", {0},false },
    { "gyro_x_alt.csv",     "alt_agl_m,gyro_x_dps\r\n", {0},false },
    { "gyro_y_alt.csv",     "alt_agl_m,gyro_y_dps\r\n", {0},false },
    { "gyro_z_alt.csv",     "alt_agl_m,gyro_z_dps\r\n", {0},false },
    { "pm1_alt.csv",        "alt_agl_m,pm1_0_ugm3\r\n", {0},false },
    { "pm25_alt.csv",       "alt_agl_m,pm2_5_ugm3\r\n", {0},false },
    { "pm10_alt.csv",       "alt_agl_m,pm10_ugm3\r\n", {0},false },
    { "airqual_alt.csv",    "alt_agl_m,aq_response\r\n", {0},false },
    { "light_alt.csv",      "alt_agl_m,fused_lux\r\n", {0},false },
    { "windspd_alt.csv",    "alt_agl_m,wind_speed_mps\r\n", {0},false },
    { "winddir_alt.csv",    "alt_agl_m,wind_dir_deg\r\n", {0},false },
    { "gnss_track.csv",     "timestamp_ms,lat,lon,alt_m,speed_mps,course_deg,sats\r\n", {0},false }
};

static bool     sInit   = false;
static bool     sOpen   = false;
static FATFS    sFs;
static uint32_t sSyncMs = 0;
static char     sLn[DL_LINE_BUF];

static DlResult WrRow(uint8_t i)
{
    UINT bw, len = (UINT)strlen(sLn);
    FRESULT r = f_write(&sF[i].fil, sLn, len, &bw);
    return (r == FR_OK && bw == len) ? DL_OK : DL_ERR_FILE;
}

/**************************************************************************************************
 * Lifecycle
 **************************************************************************************************/

DlResult DataLogger_Init(void)
{
    sInit = false; sOpen = false; sSyncMs = 0;
    if (f_mount(&sFs, "", 1) != FR_OK) return DL_ERR_FS;
    sInit = true;
    return DL_OK;
}

bool DataLogger_IsInitialised(void) { return sInit; }

DlResult DataLogger_Open(void)
{
    if (!sInit) return DL_ERR_NOT_INIT;
    for (uint8_t i = 0; i < DL_NFILES; i++)
    {
        FRESULT r = f_open(&sF[i].fil, sF[i].name, FA_WRITE | FA_OPEN_ALWAYS);
        if (r != FR_OK) return DL_ERR_FILE;
        if (f_size(&sF[i].fil) == 0)
        {
            UINT bw, hl = (UINT)strlen(sF[i].hdr);
            if (f_write(&sF[i].fil, sF[i].hdr, hl, &bw) != FR_OK) return DL_ERR_FILE;
        }
        else
        {
            f_lseek(&sF[i].fil, f_size(&sF[i].fil));
        }
        sF[i].open = true;
    }
    sOpen = true;
    return DL_OK;
}

DlResult DataLogger_Sync(uint32_t now_ms)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    if ((now_ms - sSyncMs) < DATALOG_FLUSH_INTERVAL_MS) return DL_OK;
    sSyncMs = now_ms;
    for (uint8_t i = 0; i < DL_NFILES; i++)
        if (sF[i].open) f_sync(&sF[i].fil);
    return DL_OK;
}

DlResult DataLogger_Close(void)
{
    for (uint8_t i = 0; i < DL_NFILES; i++)
        if (sF[i].open) { f_close(&sF[i].fil); sF[i].open = false; }
    sOpen = false;
    f_mount(NULL, "", 0);
    sInit = false;
    return DL_OK;
}

/**************************************************************************************************
 * 1 CSV = 1 graph row writers
 **************************************************************************************************/

DlResult DataLogger_LogAltTime(uint32_t ts, float alt)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%lu,%.2f\r\n", (unsigned long)ts, (double)alt);
    return WrRow(F_ALT_TIME);
}

DlResult DataLogger_LogPressure(float alt, uint32_t pa)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%lu\r\n", (double)alt, (unsigned long)pa);
    return WrRow(F_PRESSURE);
}

DlResult DataLogger_LogTemp(float alt, int32_t tc)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%ld\r\n", (double)alt, (long)tc);
    return WrRow(F_TEMP);
}

DlResult DataLogger_LogAccelX(float alt, float v)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%.4f\r\n", (double)alt, (double)v);
    return WrRow(F_ACCEL_X);
}

DlResult DataLogger_LogAccelY(float alt, float v)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%.4f\r\n", (double)alt, (double)v);
    return WrRow(F_ACCEL_Y);
}

DlResult DataLogger_LogAccelZ(float alt, float v)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%.4f\r\n", (double)alt, (double)v);
    return WrRow(F_ACCEL_Z);
}

DlResult DataLogger_LogGyroX(float alt, float v)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%.2f\r\n", (double)alt, (double)v);
    return WrRow(F_GYRO_X);
}

DlResult DataLogger_LogGyroY(float alt, float v)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%.2f\r\n", (double)alt, (double)v);
    return WrRow(F_GYRO_Y);
}

DlResult DataLogger_LogGyroZ(float alt, float v)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%.2f\r\n", (double)alt, (double)v);
    return WrRow(F_GYRO_Z);
}

DlResult DataLogger_LogPM1(float alt, uint16_t v)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%u\r\n", (double)alt, (unsigned)v);
    return WrRow(F_PM1);
}

DlResult DataLogger_LogPM25(float alt, uint16_t v)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%u\r\n", (double)alt, (unsigned)v);
    return WrRow(F_PM25);
}

DlResult DataLogger_LogPM10(float alt, uint16_t v)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%u\r\n", (double)alt, (unsigned)v);
    return WrRow(F_PM10);
}

DlResult DataLogger_LogAirQual(float alt, float resp)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%.2f\r\n", (double)alt, (double)resp);
    return WrRow(F_AIRQUAL);
}

DlResult DataLogger_LogLight(float alt, float lux)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%.1f\r\n", (double)alt, (double)lux);
    return WrRow(F_LIGHT);
}

DlResult DataLogger_LogWindSpd(float alt, float spd)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%.2f\r\n", (double)alt, (double)spd);
    return WrRow(F_WINDSPD);
}

DlResult DataLogger_LogWindDir(float alt, float dir)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%.2f,%.1f\r\n", (double)alt, (double)dir);
    return WrRow(F_WINDDIR);
}

DlResult DataLogger_LogGnss(uint32_t ts, double lat, double lon,
                            float alt, float spd, float crs, uint8_t sat)
{
    if (!sOpen) return DL_ERR_NOT_INIT;
    snprintf(sLn, DL_LINE_BUF, "%lu,%.6f,%.6f,%.1f,%.2f,%.1f,%u\r\n",
             (unsigned long)ts, lat, lon, (double)alt, (double)spd, (double)crs, (unsigned)sat);
    return WrRow(F_GNSS);
}
