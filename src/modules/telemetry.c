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
 * File:        telemetry.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   LoRa downlink telemetry module for CONSCIENTIA.
 *
 * Description:
 *   Formats sensor data as a compact ASCII packet and transmits via the
 *   LoRa E32-433T3D driver in transparent UART mode.
 *
 *   Packet structure (provisional — TODO: agree with GS team):
 *     $$IGNI,<seq>,<ts_ms>,<baro_temp_cdegc>,<baro_pres_pa>,<baro_alt_m>,
 *            <ax_g>,<ay_g>,<az_g>,<gx_dps>,<gy_dps>,<gz_dps>,
 *            <light1_lux>,<light2_lux>,<light3_lux>,
 *            <pm25_atm>,<pm10_atm>,<mq135_raw>,
 *            <lat_deg>,<lon_deg>,<gnss_alt_m>,<speed_mps>,<sats>,
 *            <vmask>*<NMEA-style XOR checksum>\r\n
 *
 *   The module checks the LoRa AUX pin before every transmission.
 *   If AUX is not ready the packet is silently dropped for that interval.
 *
 * \todo    Fix the final packet schema with the GS software team.
 * \todo    Consider adding a CRC16 instead of XOR checksum for robustness.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include "telemetry.h"
#include "drivers/lora_e32.h"
#include "config.h"

#include <stdio.h>
#include <string.h>
#include <stddef.h>

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool     sInitOk     = false;
static uint32_t sLastTxMs   = 0;
static uint32_t sSeqNum     = 0;

static char     sPktBuf[TELEMETRY_PKT_MAX_LEN + 1U];

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/
static uint8_t CalcXor(const char *buf, int len);

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

TmResult Telemetry_Init(void)
{
    sInitOk   = false;
    sLastTxMs = 0;
    sSeqNum   = 0;

    if (LoraE32_Init() != LORA_OK)
    {
        return TM_ERR_LORA;
    }

    sInitOk = true;
    return TM_OK;
}

bool Telemetry_IsInitialised(void)
{
    return sInitOk;
}

TmResult Telemetry_Update(const SmData *data, uint32_t now_ms)
{
    uint8_t  pkt_len;
    TmResult res;

    if (!sInitOk)   { return TM_ERR_NOT_INIT; }
    if (data == NULL) { return TM_ERR_NOT_INIT; }

    if ((now_ms - sLastTxMs) < TELEMETRY_TX_INTERVAL_MS)
    {
        return TM_OK;
    }

    /* Skip transmission if LoRa module is still busy */
    if (!LoraE32_IsAuxReady())
    {
        return TM_OK;
    }

    res = Telemetry_FormatPacket(data, sPktBuf, sizeof(sPktBuf), &pkt_len);
    if (res != TM_OK)
    {
        return res;
    }

    if (LoraE32_Send((const uint8_t *)sPktBuf, pkt_len) != LORA_OK)
    {
        return TM_ERR_LORA;
    }

    sLastTxMs = now_ms;
    sSeqNum++;

    return TM_OK;
}

TmResult Telemetry_FormatPacket(const SmData *data,
                                char *buf, uint8_t buf_len,
                                uint8_t *pkt_len)
{
    if (buf == NULL || pkt_len == NULL || data == NULL)
    {
        return TM_ERR_FORMAT;
    }

    /* Build validity mask (same bit mapping as data_logger) */
    uint8_t vmask = 0;
    if (data->baro_valid)   { vmask |= (1U << 0); }
    if (data->imu_valid)    { vmask |= (1U << 1); }
    if (data->light1_valid) { vmask |= (1U << 2); }
    if (data->light2_valid) { vmask |= (1U << 3); }
    if (data->light3_valid) { vmask |= (1U << 4); }
    if (data->pm_valid)     { vmask |= (1U << 5); }
    if (data->aq_valid)     { vmask |= (1U << 6); }
    if (data->gnss_valid)   { vmask |= (1U << 7); }

    /*
     * Format packet body (without trailing checksum first so we can XOR it).
     * \todo  Finalise field order and units with ground-station team.
     */
    char body[TELEMETRY_PKT_MAX_LEN];
    int body_len = snprintf(body, sizeof(body),
        "%s%lu,%lu,%ld,%lu,%.1f,"
        "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,"
        "%.1f,%.1f,%.1f,"
        "%u,%u,%u,"
        "%.6f,%.6f,%.1f,%.2f,%u,"
        "%u",
        TELEMETRY_SYNC,
        (unsigned long)sSeqNum,
        (unsigned long)data->imu_timestamp_ms,
        /* baro */
        (long)data->baro.temperature_cdegc,
        (unsigned long)data->baro.pressure_pa,
        data->baro_altitude_m,
        /* imu */
        data->imu.accel_x_g, data->imu.accel_y_g, data->imu.accel_z_g,
        data->imu.gyro_x_dps, data->imu.gyro_y_dps, data->imu.gyro_z_dps,
        /* light */
        data->light1_lux, data->light2_lux, data->light3_lux,
        /* pm + mq135 */
        data->pm.pm2_5_atm, data->pm.pm10_atm, data->aq.raw_adc,
        /* gnss */
        data->gnss.latitude_deg, data->gnss.longitude_deg,
        data->gnss.altitude_m, data->gnss.speed_mps,
        (unsigned)data->gnss.satellites,
        /* validity */
        (unsigned)vmask
    );

    if (body_len <= 0 || body_len >= (int)sizeof(body))
    {
        return TM_ERR_FORMAT;
    }

    /* XOR checksum over body bytes (skip the leading "$$") */
    uint8_t cs = CalcXor(body + 2, body_len - 2);

    /* Append checksum and terminator */
    int total = snprintf(buf, buf_len, "%s*%02X\r\n", body, (unsigned)cs);

    if (total <= 0 || total >= (int)buf_len)
    {
        return TM_ERR_FORMAT;
    }

    *pkt_len = (uint8_t)total;
    return TM_OK;
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

/**
 * \brief   Computes the XOR checksum of a byte buffer.
 */
static uint8_t CalcXor(const char *buf, int len)
{
    uint8_t cs = 0;
    for (int i = 0; i < len; i++)
    {
        cs ^= (uint8_t)buf[i];
    }
    return cs;
}
