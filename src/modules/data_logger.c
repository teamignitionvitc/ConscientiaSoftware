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
 *   Multi-file FAT32 CSV data logger for CONSCIENTIA.
 *
 * Description:
 *   Uses FatFs to manage 6 CSV files on a FAT32-formatted SD card.
 *   Each file has a fixed header row and per-sample data rows.
 *   Every row contains timestamp_ms and alt_agl_m for post-flight
 *   correlation by GLANCE.
 *
 *   Row formatting uses snprintf into a line buffer then f_write.
 *
 * \todo  Add file-size limit and rollover for very long ground tests.
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

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include "data_logger.h"
#include "config.h"
#include "ff.h"

#include <stdio.h>
#include <string.h>

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/

#define DL_LINE_BUF_SIZE   (256U)
#define DL_FILE_COUNT      (6U)

/**************************************************************************************************
 * File Descriptors
 **************************************************************************************************/

typedef struct
{
    const char *filename;
    const char *header;
    FIL         fil;
    bool        open;
} DlFile;

static DlFile sFiles[DL_FILE_COUNT] =
{
    /* 0 — mission.csv */
    {
        "mission.csv",
        "timestamp_ms,alt_agl_m,baro_pres_pa,baro_temp_cdegc,"
        "gnss_lat,gnss_lon,gnss_alt_m,gnss_speed,gnss_course,gnss_sats,"
        "flight_phase\r\n",
        {0}, false
    },
    /* 1 — imu.csv */
    {
        "imu.csv",
        "timestamp_ms,alt_agl_m,"
        "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,imu_temp_c\r\n",
        {0}, false
    },
    /* 2 — particulate.csv */
    {
        "particulate.csv",
        "timestamp_ms,alt_agl_m,"
        "pm1_0_atm,pm2_5_atm,pm10_atm,paired_valid\r\n",
        {0}, false
    },
    /* 3 — air_quality.csv */
    {
        "air_quality.csv",
        "timestamp_ms,alt_agl_m,"
        "raw_adc,adc_mv,sensor_mv,rs_ohms,rs_r0,response,valid\r\n",
        {0}, false
    },
    /* 4 — light.csv */
    {
        "light.csv",
        "timestamp_ms,alt_agl_m,"
        "lux1,lux2,lux3,fused_lux,spread_lux,sensors_used,"
        "v1,v2,v3,motion\r\n",
        {0}, false
    },
    /* 5 — wind.csv */
    {
        "wind.csv",
        "timestamp_ms,alt_agl_m,"
        "gnd_speed,gnd_course,gnd_vn,gnd_ve,"
        "wind_speed,wind_dir,wind_vn,wind_ve,"
        "quality,gyro_mag,sats\r\n",
        {0}, false
    }
};

enum { F_MISSION = 0, F_IMU, F_PARTICULATE, F_AIRQUALITY, F_LIGHT, F_WIND };

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool      sInitOk    = false;
static bool      sFilesOpen = false;
static FATFS     sFatFs;
static uint32_t  sLastSyncMs = 0;
static char      sLineBuf[DL_LINE_BUF_SIZE];

/**************************************************************************************************
 * Static Helpers
 **************************************************************************************************/

static DlResult WriteRow(uint8_t idx)
{
    UINT bw;
    UINT len = (UINT)strlen(sLineBuf);
    FRESULT fr = f_write(&sFiles[idx].fil, sLineBuf, len, &bw);
    if (fr != FR_OK || bw != len)
    {
        return DL_ERR_FILE;
    }
    return DL_OK;
}

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

DlResult DataLogger_Init(void)
{
    sInitOk    = false;
    sFilesOpen = false;
    sLastSyncMs = 0;

    /* Mount volume 0 (the SD card).  f_mount triggers disk_initialize
     * internally, which calls SdCard_Init() via our diskio.c adapter. */
    FRESULT fr = f_mount(&sFatFs, "", 1);  /* 1 = mount now */
    if (fr != FR_OK)
    {
        return DL_ERR_FS;
    }

    sInitOk = true;
    return DL_OK;
}

bool DataLogger_IsInitialised(void)
{
    return sInitOk;
}

DlResult DataLogger_Open(void)
{
    if (!sInitOk) { return DL_ERR_NOT_INIT; }

    for (uint8_t i = 0; i < DL_FILE_COUNT; i++)
    {
        /* Open for writing, append if the file already exists */
        FRESULT fr = f_open(&sFiles[i].fil, sFiles[i].filename,
                            FA_WRITE | FA_OPEN_ALWAYS);
        if (fr != FR_OK)
        {
            return DL_ERR_FILE;
        }

        /* If file is new (size == 0), write the CSV header */
        if (f_size(&sFiles[i].fil) == 0)
        {
            UINT bw;
            UINT hlen = (UINT)strlen(sFiles[i].header);
            fr = f_write(&sFiles[i].fil, sFiles[i].header, hlen, &bw);
            if (fr != FR_OK || bw != hlen)
            {
                return DL_ERR_FILE;
            }
        }
        else
        {
            /* Seek to end for appending */
            f_lseek(&sFiles[i].fil, f_size(&sFiles[i].fil));
        }

        sFiles[i].open = true;
    }

    sFilesOpen = true;
    return DL_OK;
}

DlResult DataLogger_Sync(uint32_t now_ms)
{
    if (!sInitOk || !sFilesOpen) { return DL_ERR_NOT_INIT; }

    if ((now_ms - sLastSyncMs) < DATALOG_FLUSH_INTERVAL_MS)
    {
        return DL_OK;
    }
    sLastSyncMs = now_ms;

    for (uint8_t i = 0; i < DL_FILE_COUNT; i++)
    {
        if (sFiles[i].open)
        {
            f_sync(&sFiles[i].fil);
        }
    }
    return DL_OK;
}

DlResult DataLogger_Close(void)
{
    for (uint8_t i = 0; i < DL_FILE_COUNT; i++)
    {
        if (sFiles[i].open)
        {
            f_close(&sFiles[i].fil);
            sFiles[i].open = false;
        }
    }
    sFilesOpen = false;

    f_mount(NULL, "", 0);   /* Unmount */
    sInitOk = false;
    return DL_OK;
}

/* ── Per-dataset CSV row writers ─────────────────────────────────────── */

DlResult DataLogger_LogMission(const SmData *sm, float alt_agl, uint8_t phase,
                               uint32_t ts_ms)
{
    if (!sFilesOpen) { return DL_ERR_NOT_INIT; }
    if (sm == NULL)  { return DL_ERR_FILE; }

    snprintf(sLineBuf, DL_LINE_BUF_SIZE,
        "%lu,%.2f,%lu,%ld,"          /* ts, alt, pres, temp */
        "%.6f,%.6f,%.1f,%.2f,%.1f,%u,"  /* gnss */
        "%u\r\n",                     /* phase */
        (unsigned long)ts_ms,
        (double)alt_agl,
        (unsigned long)sm->baro.pressure_pa,
        (long)sm->baro.temperature_cdegc,
        sm->gnss.latitude_deg,
        sm->gnss.longitude_deg,
        (double)sm->gnss.altitude_m,
        (double)sm->gnss.speed_mps,
        (double)sm->gnss.course_deg,
        (unsigned)sm->gnss.satellites,
        (unsigned)phase);

    return WriteRow(F_MISSION);
}

DlResult DataLogger_LogImu(const SmData *sm, float alt_agl, uint32_t ts_ms)
{
    if (!sFilesOpen) { return DL_ERR_NOT_INIT; }
    if (sm == NULL)  { return DL_ERR_FILE; }

    snprintf(sLineBuf, DL_LINE_BUF_SIZE,
        "%lu,%.2f,"
        "%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f\r\n",
        (unsigned long)ts_ms,
        (double)alt_agl,
        (double)sm->imu.accel_x_g,
        (double)sm->imu.accel_y_g,
        (double)sm->imu.accel_z_g,
        (double)sm->imu.gyro_x_dps,
        (double)sm->imu.gyro_y_dps,
        (double)sm->imu.gyro_z_dps,
        (double)sm->imu.temperature_c);

    return WriteRow(F_IMU);
}

DlResult DataLogger_LogParticulate(const PaData *pa)
{
    if (!sFilesOpen) { return DL_ERR_NOT_INIT; }
    if (pa == NULL)  { return DL_ERR_FILE; }

    snprintf(sLineBuf, DL_LINE_BUF_SIZE,
        "%lu,%.2f,%u,%u,%u,%u\r\n",
        (unsigned long)pa->pm_timestamp_ms,
        (double)pa->altitude_m,
        (unsigned)pa->pm1_0_atm,
        (unsigned)pa->pm2_5_atm,
        (unsigned)pa->pm10_atm,
        (unsigned)pa->paired_valid);

    return WriteRow(F_PARTICULATE);
}

DlResult DataLogger_LogAirQuality(const AqData *aq, float alt_agl,
                                  uint32_t ts_ms)
{
    if (!sFilesOpen) { return DL_ERR_NOT_INIT; }
    if (aq == NULL)  { return DL_ERR_FILE; }

    snprintf(sLineBuf, DL_LINE_BUF_SIZE,
        "%lu,%.2f,%u,%.1f,%.1f,%.1f,%.4f,%.2f,%u\r\n",
        (unsigned long)ts_ms,
        (double)alt_agl,
        (unsigned)aq->raw_adc,
        (double)aq->adc_voltage_mv,
        (double)aq->sensor_voltage_mv,
        (double)aq->rs_ohms,
        (double)aq->rs_over_r0,
        (double)aq->air_quality_response,
        (unsigned)aq->valid);

    return WriteRow(F_AIRQUALITY);
}

DlResult DataLogger_LogLight(const LfData *lf)
{
    if (!sFilesOpen) { return DL_ERR_NOT_INIT; }
    if (lf == NULL)  { return DL_ERR_FILE; }

    snprintf(sLineBuf, DL_LINE_BUF_SIZE,
        "%lu,%.2f,"
        "%.1f,%.1f,%.1f,%.1f,%.1f,%u,"
        "%u,%u,%u,%u\r\n",
        (unsigned long)lf->light_timestamp_ms,
        (double)lf->altitude_m,
        (double)lf->lux1,
        (double)lf->lux2,
        (double)lf->lux3,
        (double)lf->fused_lux,
        (double)lf->spread_lux,
        (unsigned)lf->sensors_used,
        (unsigned)lf->valid1,
        (unsigned)lf->valid2,
        (unsigned)lf->valid3,
        (unsigned)lf->motion);

    return WriteRow(F_LIGHT);
}

DlResult DataLogger_LogWind(const WiData *wi)
{
    if (!sFilesOpen) { return DL_ERR_NOT_INIT; }
    if (wi == NULL)  { return DL_ERR_FILE; }

    snprintf(sLineBuf, DL_LINE_BUF_SIZE,
        "%lu,%.2f,"
        "%.2f,%.1f,%.3f,%.3f,"
        "%.2f,%.1f,%.3f,%.3f,"
        "%u,%.1f,%u\r\n",
        (unsigned long)wi->timestamp_ms,
        (double)wi->altitude_agl_m,
        (double)wi->gnd_speed_mps,
        (double)wi->gnd_course_deg,
        (double)wi->gnd_vn_mps,
        (double)wi->gnd_ve_mps,
        (double)wi->wind_speed_mps,
        (double)wi->wind_dir_deg,
        (double)wi->wind_vn_mps,
        (double)wi->wind_ve_mps,
        (unsigned)wi->quality,
        (double)wi->gyro_mag_dps,
        (unsigned)wi->satellites);

    return WriteRow(F_WIND);
}
