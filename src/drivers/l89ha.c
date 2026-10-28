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
 * File:        l89ha.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Source file for the L89HA GNSS module driver.
 *
 * Description:
 *   NMEA sentence reception and parsing over the shared Pico SDK I2C0 bus.
 *   The physical module responds at command address 0x50. For the R2.0-style
 *   transport, commands are written to 0x50 and response data is read at 0x54.
 *
 *   Sentences parsed:
 *     $GNRMC — time, fix validity, lat, lon, ground speed, course
 *     $GNGGA — lat, lon, fix quality, satellites, altitude
 *
 *   Both sentences are validated with the standard NMEA XOR checksum
 *   (*HH suffix).  Malformed or incomplete data is silently discarded
 *   and the last valid fix is preserved.
 *
 * \todo    Add $GNGSA (DOP/fix type) parsing if required by flight software.
 * \todo    Consider DMA UART if main-loop jitter > 50 ms.
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
#include "l89ha.h"
#include "config.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/
#define SENTENCE_BUF_LEN       (L89HA_NMEA_MAX_LEN + 1U)
#define GNSS_I2C_CMD_LEN       (8U)
#define GNSS_I2C_CHUNK_MAX     (256U)
#define GNSS_I2C_DELAY_MS      (10U)
#define GNSS_TX_LEN_OFFSET     (0x0008U)
#define GNSS_TX_BUF_OFFSET     (0x2000U)
#define GNSS_CR_CMD            (0xAA51U)

/* Knots to m/s conversion */
#define KNOTS_TO_MPS        (0.514444f)

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool      sInitOk   = false;
static bool      sHasFix   = false;
static L89haFix  sLatestFix;

/* Sentence assembly buffer */
static char      sBuf[SENTENCE_BUF_LEN];
static uint8_t   sBufIdx = 0;
static bool      sInSentence = false;

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/
static bool     ValidateChecksum(const char *sentence);
static void     ParseRMC(const char *sentence);
static void     ParseGGA(const char *sentence);
static double   ParseLatLon(const char *field, const char *hemi, bool is_lon);
static uint8_t  FieldCopy(const char *sentence, uint8_t field_num,
                           char *out, uint8_t out_len);
static bool     ReadI2cNmea(uint8_t *buf, uint16_t max_len, uint16_t *read_len);
static void     FeedNmeaByte(char c, L89haResult *last_result);
static void     BuildReadCommand(uint16_t offset, uint32_t length, uint8_t cmd[GNSS_I2C_CMD_LEN]);

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

L89haResult L89HA_Init(void)
{
    sInitOk  = false;
    sHasFix  = false;
    sBufIdx  = 0;
    sInSentence = false;
    memset(&sLatestFix, 0, sizeof(sLatestFix));

    /* I2C0 and its pins are initialised centrally by SensorManager_Init(). */
    sInitOk = true;
    return L89HA_OK;
}

bool L89HA_IsInitialised(void)
{
    return sInitOk;
}

L89haResult L89HA_Poll(void)
{
    uint8_t raw[GNSS_I2C_CHUNK_MAX];
    uint16_t received = 0U;
    L89haResult last_result = L89HA_ERR_NO_DATA;

    if (!sInitOk)
    {
        return L89HA_ERR_NOT_INIT;
    }

    if (!ReadI2cNmea(raw, sizeof(raw), &received))
    {
        return L89HA_ERR_I2C;
    }

    for (uint16_t i = 0U; i < received; ++i)
    {
        FeedNmeaByte((char)raw[i], &last_result);
    }

    return last_result;
}

L89haResult L89HA_GetFix(L89haFix *fix)
{
    if (!sInitOk)
    {
        return L89HA_ERR_NOT_INIT;
    }
    if (fix == NULL)
    {
        return L89HA_ERR_NO_FIX;
    }
    if (!sHasFix)
    {
        return L89HA_ERR_NO_FIX;
    }

    *fix = sLatestFix;
    return L89HA_OK;
}

bool L89HA_HasFix(void)
{
    return sHasFix;
}


static void BuildReadCommand(uint16_t offset, uint32_t length, uint8_t cmd[GNSS_I2C_CMD_LEN])
{
    uint32_t word0 = ((uint32_t)GNSS_CR_CMD << 16) | (uint32_t)offset;
    cmd[0] = (uint8_t)(word0 & 0xFFU);
    cmd[1] = (uint8_t)((word0 >> 8) & 0xFFU);
    cmd[2] = (uint8_t)((word0 >> 16) & 0xFFU);
    cmd[3] = (uint8_t)((word0 >> 24) & 0xFFU);
    cmd[4] = (uint8_t)(length & 0xFFU);
    cmd[5] = (uint8_t)((length >> 8) & 0xFFU);
    cmd[6] = (uint8_t)((length >> 16) & 0xFFU);
    cmd[7] = (uint8_t)((length >> 24) & 0xFFU);
}

static bool ReadI2cNmea(uint8_t *buf, uint16_t max_len, uint16_t *read_len)
{
    uint8_t cmd[GNSS_I2C_CMD_LEN];
    uint8_t len_bytes[4];
    uint32_t available;
    uint16_t request_len;

    if (buf == NULL || read_len == NULL || max_len == 0U)
    {
        return false;
    }
    *read_len = 0U;

    /* Quectel multi-slave guidance: address the command endpoint first. */
    if (i2c_write_timeout_us(I2C0_INSTANCE, L89HA_I2C_ADDR, NULL, 0, false, 10000) < 0)
    {
        return false;
    }

    BuildReadCommand(GNSS_TX_LEN_OFFSET, 4U, cmd);
    if (i2c_write_timeout_us(I2C0_INSTANCE, L89HA_I2C_ADDR, cmd, sizeof(cmd), false, 10000) != (int)sizeof(cmd))
    {
        return false;
    }
    sleep_ms(GNSS_I2C_DELAY_MS);

    if (i2c_read_timeout_us(I2C0_INSTANCE, L89HA_I2C_READ_ADDR, len_bytes, sizeof(len_bytes), false, 10000) != (int)sizeof(len_bytes))
    {
        return false;
    }

    available = (uint32_t)len_bytes[0]
              | ((uint32_t)len_bytes[1] << 8)
              | ((uint32_t)len_bytes[2] << 16)
              | ((uint32_t)len_bytes[3] << 24);

    if (available == 0U)
    {
        return true;
    }

    request_len = (available > (uint32_t)max_len) ? max_len : (uint16_t)available;
    BuildReadCommand(GNSS_TX_BUF_OFFSET, request_len, cmd);
    if (i2c_write_timeout_us(I2C0_INSTANCE, L89HA_I2C_ADDR, cmd, sizeof(cmd), false, 10000) != (int)sizeof(cmd))
    {
        return false;
    }
    sleep_ms(GNSS_I2C_DELAY_MS);

    if (i2c_read_timeout_us(I2C0_INSTANCE, L89HA_I2C_READ_ADDR, buf, request_len, false, 20000) != (int)request_len)
    {
        return false;
    }

    *read_len = request_len;
    return true;
}

static void FeedNmeaByte(char c, L89haResult *last_result)
{
    if (c == '$')
    {
        sBuf[0] = '$';
        sBufIdx = 1U;
        sInSentence = true;
        return;
    }

    if (!sInSentence)
    {
        return;
    }

    if (c == '\n' || c == '\r')
    {
        sBuf[sBufIdx] = '\0';
        sInSentence = false;

        if (!ValidateChecksum(sBuf))
        {
            *last_result = L89HA_ERR_CHECKSUM;
        }
        else if (strncmp(sBuf, "$GNRMC", 6) == 0 || strncmp(sBuf, "$GPRMC", 6) == 0)
        {
            ParseRMC(sBuf);
            *last_result = L89HA_OK;
        }
        else if (strncmp(sBuf, "$GNGGA", 6) == 0 || strncmp(sBuf, "$GPGGA", 6) == 0)
        {
            ParseGGA(sBuf);
            *last_result = L89HA_OK;
        }

        sBufIdx = 0U;
        return;
    }

    if (sBufIdx >= (SENTENCE_BUF_LEN - 1U))
    {
        sInSentence = false;
        sBufIdx = 0U;
        *last_result = L89HA_ERR_PARSE;
        return;
    }

    sBuf[sBufIdx++] = c;
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

/**
 * \brief   Validates the NMEA XOR checksum embedded in a sentence.
 * \details Checksum is XOR of all bytes between '$' and '*' exclusive.
 *          The two hex digits after '*' are the expected value.
 */
static bool ValidateChecksum(const char *sentence)
{
    const char *p  = sentence + 1;   /* Skip '$' */
    const char *cs = NULL;
    uint8_t     calc = 0;

    while (*p && *p != '*')
    {
        calc ^= (uint8_t)*p++;
    }

    if (*p != '*')
    {
        return false;   /* No checksum delimiter */
    }

    cs = p + 1;
    if (cs[0] == '\0' || cs[1] == '\0')
    {
        return false;
    }

    uint8_t rx = (uint8_t)strtol(cs, NULL, 16);
    return (calc == rx);
}

/**
 * \brief   Copies the Nth comma-delimited field from an NMEA sentence.
 * \return  Length of the field copied, 0 if field does not exist.
 */
static uint8_t FieldCopy(const char *sentence, uint8_t field_num,
                          char *out, uint8_t out_len)
{
    const char *p = sentence;
    uint8_t field = 0;

    while (*p && field < field_num)
    {
        if (*p == ',')
        {
            field++;
        }
        p++;
    }

    uint8_t idx = 0;
    while (*p && *p != ',' && *p != '*' && idx < (out_len - 1U))
    {
        out[idx++] = *p++;
    }
    out[idx] = '\0';
    return idx;
}

/**
 * \brief   Converts raw NMEA lat/lon (DDDMM.MMMM) to decimal degrees.
 */
static double ParseLatLon(const char *field, const char *hemi, bool is_lon)
{
    if (field[0] == '\0')
    {
        return 0.0;
    }

    double raw = atof(field);
    double deg;
    double min;

    if (is_lon)
    {
        deg = (int)(raw / 100.0);
        min = raw - deg * 100.0;
    }
    else
    {
        deg = (int)(raw / 100.0);
        min = raw - deg * 100.0;
    }

    double result = deg + (min / 60.0);
    if (hemi[0] == 'S' || hemi[0] == 'W')
    {
        result = -result;
    }
    return result;
}

/**
 * \brief   Parses a $GNRMC / $GPRMC sentence.
 * \details Fields: time, status, lat, N/S, lon, E/W, speed(kt), course, date,…
 */
static void ParseRMC(const char *sentence)
{
    char f[20];

    /* Field 2: status (A = valid, V = invalid) */
    FieldCopy(sentence, 2, f, sizeof(f));
    bool valid = (f[0] == 'A');

    /* Field 1: UTC time HHMMSS.ss */
    FieldCopy(sentence, 1, f, sizeof(f));
    if (f[0] != '\0')
    {
        uint32_t t = (uint32_t)atof(f);
        sLatestFix.utc_time.hours   = (uint8_t)((t / 10000U) % 100U);
        sLatestFix.utc_time.minutes = (uint8_t)((t / 100U)   % 100U);
        sLatestFix.utc_time.seconds = (uint8_t)( t            % 100U);
    }

    if (!valid)
    {
        sLatestFix.fix_valid = false;
        return;
    }

    /* Fields 3/4: latitude */
    char lat_f[16], lat_h[4];
    FieldCopy(sentence, 3, lat_f, sizeof(lat_f));
    FieldCopy(sentence, 4, lat_h, sizeof(lat_h));
    sLatestFix.latitude_deg = ParseLatLon(lat_f, lat_h, false);

    /* Fields 5/6: longitude */
    char lon_f[16], lon_h[4];
    FieldCopy(sentence, 5, lon_f, sizeof(lon_f));
    FieldCopy(sentence, 6, lon_h, sizeof(lon_h));
    sLatestFix.longitude_deg = ParseLatLon(lon_f, lon_h, true);

    /* Field 7: speed in knots */
    FieldCopy(sentence, 7, f, sizeof(f));
    if (f[0] != '\0')
    {
        sLatestFix.speed_mps = (float)atof(f) * KNOTS_TO_MPS;
    }

    /* Field 8: true course */
    FieldCopy(sentence, 8, f, sizeof(f));
    if (f[0] != '\0')
    {
        sLatestFix.course_deg = (float)atof(f);
    }

    sLatestFix.fix_valid = true;
    sHasFix              = true;
}

/**
 * \brief   Parses a $GNGGA / $GPGGA sentence.
 * \details Fields: time, lat, N/S, lon, E/W, quality, sats, HDOP, alt,…
 */
static void ParseGGA(const char *sentence)
{
    char f[20];

    /* Field 6: fix quality (0 = invalid) */
    FieldCopy(sentence, 6, f, sizeof(f));
    if (f[0] == '0' || f[0] == '\0')
    {
        return;
    }

    /* Field 7: satellites in use */
    FieldCopy(sentence, 7, f, sizeof(f));
    if (f[0] != '\0')
    {
        sLatestFix.satellites = (uint8_t)atoi(f);
    }

    /* Field 9: altitude above MSL */
    FieldCopy(sentence, 9, f, sizeof(f));
    if (f[0] != '\0')
    {
        sLatestFix.altitude_m = (float)atof(f);
    }
}
