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
 * File:        sd_card.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Source file for the Micro SD card SPI driver.
 *
 * Description:
 *   Implements the SD SPI initialisation protocol and block-level I/O over
 *   Pico SDK hardware SPI1 (SCK=GPIO10, MOSI=GPIO11, MISO=GPIO12, CS=GPIO13).
 *
 *   SD SPI init sequence:
 *     1. Hold CS high, send ≥74 dummy clock cycles to power-up.
 *     2. CMD0  (GO_IDLE_STATE) — expect R1 = 0x01.
 *     3. CMD8  (SEND_IF_COND)  — detect v2 card; check voltage range.
 *     4. ACMD41 (SD_SEND_OP_COND) with HCS=1 — loop until R1 = 0x00.
 *     5. CMD58 (READ_OCR) — read CCS bit to detect SDHC vs SDv2-SDSC.
 *     6. Switch SPI to high-speed (SPI_SD_DATA_BAUDRATE_HZ).
 *
 *   Block addresses: SDHC uses block numbers; SDv1/SDv2-SDSC uses byte
 *   addresses (sector × 512).  The SdCard_ReadBlock / SdCard_WriteBlock
 *   functions handle this transparently.
 *
 *   Sequential log layer:
 *     Maintains a 512-byte sector buffer starting at SD_LOG_START_SECTOR.
 *     Completed sectors are written immediately.  SdCard_Flush() commits
 *     a partial sector (zero-padded) without advancing the write pointer,
 *     so the next write resumes at the correct offset.
 *
 * \todo    Add FatFs diskio adapter (diskio.c) when filesystem support
 *          is required.  SdCard_ReadBlock/SdCard_WriteBlock provide the
 *          required layer.
 * \todo    Validate SD_LOG_START_SECTOR against actual card capacity.
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
#include "sd_card.h"
#include "config.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <string.h>
#include <stddef.h>

/**************************************************************************************************
 * SD SPI Command Opcodes
 **************************************************************************************************/
#define CMD0            (0x40U | 0U)    /* GO_IDLE_STATE             */
#define CMD8            (0x40U | 8U)    /* SEND_IF_COND              */
#define CMD17           (0x40U | 17U)   /* READ_SINGLE_BLOCK         */
#define CMD24           (0x40U | 24U)   /* WRITE_BLOCK               */
#define CMD55           (0x40U | 55U)   /* APP_CMD prefix            */
#define CMD58           (0x40U | 58U)   /* READ_OCR                  */
#define ACMD41          (0x40U | 41U)   /* SD_SEND_OP_COND           */

#define DATA_START_TOKEN    (0xFEU)
#define DATA_RESPONSE_OK    (0x05U)     /* Mask 0x1F == 0x05 → accepted */

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/

/* SD_LOG_START_SECTOR is defined in config.h. */

/** SPI command response timeout (byte poll iterations). */
#define SD_RESP_TIMEOUT         (500U)

/** ACMD41 retry limit during card init. */
#define SD_INIT_RETRIES         (2000U)

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool         sInitOk   = false;
static SdCardType   sCardType = SD_TYPE_UNKNOWN;

/* Sequential log stream state */
static bool         sIsOpen   = false;
static uint32_t     sCurrentSector;
static uint16_t     sBufOffset;
static uint8_t      sSectorBuf[SD_BLOCK_SIZE];

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/
static void     CsAssert(void);
static void     CsDeassert(void);
static uint8_t  SpiByte(uint8_t out);
static uint8_t  SendCommand(uint8_t cmd, uint32_t arg);
static bool     WaitNotBusy(void);
static bool     WaitToken(uint8_t token);

/**************************************************************************************************
 * Function Definitions — Block Layer
 **************************************************************************************************/

SdCardResult SdCard_Init(void)
{
    uint8_t r1;
    uint32_t retries;

    sInitOk   = false;
    sCardType = SD_TYPE_UNKNOWN;

    /* Initialise SPI1 at low speed for card power-up */
    spi_init(SPI1_INSTANCE, SPI_SD_INIT_BAUDRATE_HZ);
    spi_set_format(SPI1_INSTANCE, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_SPI1_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI1_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI1_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_SD_CS);
    gpio_set_dir(PIN_SD_CS, GPIO_OUT);
    gpio_put(PIN_SD_CS, 1);

    /* ≥74 dummy clock cycles with CS high */
    sleep_ms(1);
    for (uint8_t i = 0; i < 10; i++)
    {
        SpiByte(0xFFU);
    }

    /* CMD0 — reset card into SPI mode */
    r1 = SendCommand(CMD0, 0x00000000U);
    if (r1 != 0x01U)
    {
        return SD_ERR_INIT;
    }

    /* CMD8 — check for v2 card (arg: VHS=1 (3.3V), check pattern=0xAA) */
    r1 = SendCommand(CMD8, 0x000001AAU);
    bool is_v2 = (r1 == 0x01U);

    if (is_v2)
    {
        /* Read trailing 4 bytes of R7 response */
        uint8_t buf[4];
        for (uint8_t i = 0; i < 4; i++)
        {
            buf[i] = SpiByte(0xFFU);
        }
        /* Validate voltage range and echo pattern */
        if ((buf[3] != 0xAAU) || ((buf[2] & 0x0FU) != 0x01U))
        {
            CsDeassert();
            return SD_ERR_INIT;
        }
    }

    CsDeassert();

    /* ACMD41 — activate card init, HCS=1 for SDHC support */
    retries = 0;
    do
    {
        SendCommand(CMD55, 0x00000000U);
        CsDeassert();
        r1 = SendCommand(ACMD41, is_v2 ? 0x40000000U : 0x00000000U);
        CsDeassert();
        retries++;
    }
    while (r1 != 0x00U && retries < SD_INIT_RETRIES);

    if (r1 != 0x00U)
    {
        return SD_ERR_TIMEOUT;
    }

    /* CMD58 — read OCR to detect SDHC */
    r1 = SendCommand(CMD58, 0x00000000U);
    if (r1 == 0x00U)
    {
        uint8_t ocr[4];
        for (uint8_t i = 0; i < 4; i++)
        {
            ocr[i] = SpiByte(0xFFU);
        }
        CsDeassert();

        if (is_v2 && (ocr[0] & 0x40U))
        {
            sCardType = SD_TYPE_SDHC;
        }
        else if (is_v2)
        {
            sCardType = SD_TYPE_SDv2;
        }
        else
        {
            sCardType = SD_TYPE_SDv1;
        }
    }
    else
    {
        CsDeassert();
        sCardType = SD_TYPE_SDv1;
    }

    /* Switch to high-speed SPI */
    spi_set_baudrate(SPI1_INSTANCE, SPI_SD_DATA_BAUDRATE_HZ);

    sInitOk = true;
    return SD_OK;
}

bool SdCard_IsInitialised(void)
{
    return sInitOk;
}

SdCardType SdCard_GetType(void)
{
    return sCardType;
}

SdCardResult SdCard_ReadBlock(uint32_t sector, uint8_t *buf)
{
    uint8_t r1;

    if (!sInitOk)
    {
        return SD_ERR_NOT_INIT;
    }

    /* SDv1/SDv2-SDSC use byte address */
    uint32_t addr = (sCardType == SD_TYPE_SDHC) ? sector : (sector * SD_BLOCK_SIZE);

    r1 = SendCommand(CMD17, addr);
    if (r1 != 0x00U)
    {
        CsDeassert();
        return SD_ERR_READ;
    }

    if (!WaitToken(DATA_START_TOKEN))
    {
        CsDeassert();
        return SD_ERR_TIMEOUT;
    }

    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++)
    {
        buf[i] = SpiByte(0xFFU);
    }

    /* Discard 2-byte CRC */
    SpiByte(0xFFU);
    SpiByte(0xFFU);

    CsDeassert();
    return SD_OK;
}

SdCardResult SdCard_WriteBlock(uint32_t sector, const uint8_t *buf)
{
    uint8_t response;

    if (!sInitOk)
    {
        return SD_ERR_NOT_INIT;
    }

    uint32_t addr = (sCardType == SD_TYPE_SDHC) ? sector : (sector * SD_BLOCK_SIZE);

    uint8_t r1 = SendCommand(CMD24, addr);
    if (r1 != 0x00U)
    {
        CsDeassert();
        return SD_ERR_WRITE;
    }

    SpiByte(0xFFU);                     /* One byte gap before token */
    SpiByte(DATA_START_TOKEN);

    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++)
    {
        SpiByte(buf[i]);
    }

    /* Dummy CRC */
    SpiByte(0xFFU);
    SpiByte(0xFFU);

    /* Data response token */
    response = SpiByte(0xFFU) & 0x1FU;
    if (response != DATA_RESPONSE_OK)
    {
        CsDeassert();
        return SD_ERR_WRITE;
    }

    /* Wait for card to finish programming */
    if (!WaitNotBusy())
    {
        CsDeassert();
        return SD_ERR_TIMEOUT;
    }

    CsDeassert();
    return SD_OK;
}

/**************************************************************************************************
 * Function Definitions — Sequential Byte-Stream Log Layer
 **************************************************************************************************/

SdCardResult SdCard_Open(void)
{
    if (!sInitOk)
    {
        return SD_ERR_NOT_INIT;
    }

    sCurrentSector = SD_LOG_START_SECTOR;
    sBufOffset     = 0;
    memset(sSectorBuf, 0, SD_BLOCK_SIZE);
    sIsOpen = true;

    return SD_OK;
}

SdCardResult SdCard_Write(const uint8_t *data, uint16_t len)
{
    if (!sInitOk)   { return SD_ERR_NOT_INIT; }
    if (!sIsOpen)   { return SD_ERR_NOT_OPEN; }
    if (data == NULL || len == 0) { return SD_OK; }

    uint16_t remaining = len;
    uint16_t src_idx   = 0;

    while (remaining > 0)
    {
        uint16_t space = SD_BLOCK_SIZE - sBufOffset;
        uint16_t chunk = (remaining < space) ? remaining : space;

        memcpy(&sSectorBuf[sBufOffset], &data[src_idx], chunk);
        sBufOffset += chunk;
        src_idx    += chunk;
        remaining  -= chunk;

        if (sBufOffset >= SD_BLOCK_SIZE)
        {
            SdCardResult res = SdCard_WriteBlock(sCurrentSector, sSectorBuf);
            if (res != SD_OK)
            {
                return res;
            }
            sCurrentSector++;
            sBufOffset = 0;
            memset(sSectorBuf, 0, SD_BLOCK_SIZE);
        }
    }

    return SD_OK;
}

SdCardResult SdCard_Flush(void)
{
    SdCardResult res;

    if (!sInitOk) { return SD_ERR_NOT_INIT; }
    if (!sIsOpen) { return SD_ERR_NOT_OPEN; }

    if (sBufOffset == 0)
    {
        return SD_OK;   /* Nothing pending */
    }

    /* Write partial sector zero-padded — do NOT advance sector pointer;
     * the buffer offset stays so the next write continues mid-sector. */
    uint8_t tmp[SD_BLOCK_SIZE];
    memcpy(tmp, sSectorBuf, SD_BLOCK_SIZE);

    res = SdCard_WriteBlock(sCurrentSector, tmp);
    return res;
}

SdCardResult SdCard_Close(void)
{
    SdCardResult res = SdCard_Flush();
    sIsOpen = false;
    return res;
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

static void CsAssert(void)
{
    gpio_put(PIN_SD_CS, 0);
}

static void CsDeassert(void)
{
    gpio_put(PIN_SD_CS, 1);
    SpiByte(0xFFU);     /* One extra clock to release bus */
}

static uint8_t SpiByte(uint8_t out)
{
    uint8_t in = 0;
    spi_write_read_blocking(SPI1_INSTANCE, &out, &in, 1);
    return in;
}

/**
 * \brief   Sends an SD SPI command and returns the R1 response byte.
 */
static uint8_t SendCommand(uint8_t cmd, uint32_t arg)
{
    uint8_t  crc = 0x01U;
    uint8_t  r1  = 0xFFU;
    uint32_t i;

    /* Standard CRC values for CMD0 and CMD8 */
    if (cmd == CMD0)  { crc = 0x95U; }
    if (cmd == CMD8)  { crc = 0x87U; }

    if (!WaitNotBusy())
    {
        return 0xFFU;
    }

    CsAssert();

    SpiByte(cmd);
    SpiByte((uint8_t)(arg >> 24));
    SpiByte((uint8_t)(arg >> 16));
    SpiByte((uint8_t)(arg >>  8));
    SpiByte((uint8_t)(arg >>  0));
    SpiByte(crc);

    /* Poll for non-0xFF response (up to SD_RESP_TIMEOUT bytes) */
    for (i = 0; i < SD_RESP_TIMEOUT; i++)
    {
        r1 = SpiByte(0xFFU);
        if ((r1 & 0x80U) == 0U)
        {
            break;
        }
    }

    /* CS remains asserted after command; caller must call CsDeassert() */
    return r1;
}

/**
 * \brief   Waits until the SD card releases the MISO line (not busy).
 */
static bool WaitNotBusy(void)
{
    for (uint32_t i = 0; i < SD_RESP_TIMEOUT; i++)
    {
        if (SpiByte(0xFFU) == 0xFFU)
        {
            return true;
        }
    }
    return false;
}

/**
 * \brief   Polls MISO for a specific token byte (e.g. DATA_START_TOKEN).
 */
static bool WaitToken(uint8_t token)
{
    for (uint32_t i = 0; i < SD_RESP_TIMEOUT * 4U; i++)
    {
        if (SpiByte(0xFFU) == token)
        {
            return true;
        }
    }
    return false;
}
