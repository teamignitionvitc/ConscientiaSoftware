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
 *   Non-blocking NMEA sentence reception and parsing over Pico SDK hardware
 *   UART1 (GPIO4 TX, GPIO5 RX, 9600 baud) using uart_is_readable().
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

#include "hardware/uart.h"
#include "hardware/gpio.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/
#define SENTENCE_BUF_LEN    (L89HA_NMEA_MAX_LEN + 1U)

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

    uart_init(UART1_INSTANCE, UART1_BAUDRATE);
    gpio_set_function(PIN_UART1_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART1_RX, GPIO_FUNC_UART);

    sInitOk = true;
    return L89HA_OK;
}

bool L89HA_IsInitialised(void)
{
    return sInitOk;
}

L89haResult L89HA_Poll(void)
{
    bool sentence_done = false;
    L89haResult last_result = L89HA_ERR_NO_DATA;

    if (!sInitOk)
    {
        return L89HA_ERR_NOT_INIT;
    }

    while (uart_is_readable(UART1_INSTANCE))
    {
        char c = (char)uart_getc(UART1_INSTANCE);

        if (c == '$')
        {
            /* Start of new sentence — reset buffer */
            sBuf[0]     = '$';
            sBufIdx     = 1;
            sInSentence = true;
            continue;
        }

        if (!sInSentence)
        {
            continue;
        }

        if (c == '\n' || c == '\r')
        {
            /* Sentence complete */
            sBuf[sBufIdx] = '\0';
            sInSentence   = false;
            sentence_done = true;

            if (!ValidateChecksum(sBuf))
            {
                last_result = L89HA_ERR_CHECKSUM;
                sBufIdx = 0;
                continue;
            }

            /* Dispatch by sentence type */
            if (strncmp(sBuf, "$GNRMC", 6) == 0 ||
                strncmp(sBuf, "$GPRMC", 6) == 0)
            {
                ParseRMC(sBuf);
                last_result = L89HA_OK;
            }
            else if (strncmp(sBuf, "$GNGGA", 6) == 0 ||
                     strncmp(sBuf, "$GPGGA", 6) == 0)
            {
                ParseGGA(sBuf);
                last_result = L89HA_OK;
            }

            sBufIdx = 0;
            continue;
        }

        /* Guard against buffer overflow — discard oversized sentence */
        if (sBufIdx >= (SENTENCE_BUF_LEN - 1U))
        {
            sInSentence = false;
            sBufIdx     = 0;
            last_result = L89HA_ERR_PARSE;
            continue;
        }

        sBuf[sBufIdx++] = c;
    }

    (void)sentence_done;
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
