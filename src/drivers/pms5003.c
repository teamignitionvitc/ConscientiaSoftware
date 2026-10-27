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
 * File:        pms5003.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Source file for the PMS5003 particulate matter sensor driver.
 *
 * Description:
 *   Implements PIO-based software UART reception and frame parsing for the
 *   Plantower PMS5003 on the Waveshare RP2040-Zero.
 *
 *   The PMS5003 operates in default active mode, continuously transmitting
 *   32-byte data frames at ~1 Hz over 9600-baud serial.  This driver uses
 *   PIO0 SM1 for UART RX (see config.h and uart_rx.pio).
 *
 *   Frame format (32 bytes):
 *     [0-1]   Start bytes: 0x42 0x4D
 *     [2-3]   Frame body length (should be 0x001C = 28)
 *     [4-9]   PM concentrations — standard particle (CF=1)
 *     [10-15] PM concentrations — atmospheric environment
 *     [16-27] Particle counts per 0.1 L
 *     [28-29] Reserved / version / error code
 *     [30-31] Checksum: sum of bytes [0..29]
 *
 *   Polling architecture: PMS5003_Poll() must be called frequently to drain
 *   the PIO RX FIFO before it overflows (~16 bytes deep with joined FIFOs).
 *
 * \todo    Consider DMA or PIO IRQ for guaranteed lossless reception if
 *          main-loop jitter exceeds ~15 ms.
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
#include "pms5003.h"
#include "config.h"

#include "hardware/pio.h"
#include "uart_rx.pio.h"

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/
#define FRAME_BODY_LEN      (28U)       /* Expected value in bytes [2..3]       */
#define FRAME_DATA_WORDS    (13U)       /* 13 big-endian uint16 data words      */

/**************************************************************************************************
 * Frame Parser States
 **************************************************************************************************/
typedef enum
{
    STATE_SYNC1 = 0,        /**< Waiting for 0x42.                      */
    STATE_SYNC2,            /**< Got 0x42, waiting for 0x4D.            */
    STATE_DATA              /**< Collecting remaining 30 bytes.         */
} ParseState;

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool         sInitOk    = false;
static bool         sDataValid = false;
static uint         sPioOffset = 0;

static uint8_t      sFrameBuf[PMS5003_FRAME_LEN];
static uint8_t      sFrameIdx  = 0;
static ParseState   sState     = STATE_SYNC1;

static Pms5003Data  sLatest;

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/
static void     FeedByte(uint8_t byte);
static bool     ValidateAndParse(void);
static uint16_t BE16(const uint8_t *p);

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

Pms5003Result PMS5003_Init(void)
{
    sInitOk    = false;
    sDataValid = false;
    sState     = STATE_SYNC1;
    sFrameIdx  = 0;

    /* Load PIO UART RX program */
    if (!pio_can_add_program(PIO_PMS_INSTANCE, &uart_rx_program))
    {
        return PMS5003_ERR_I2C;
    }

    sPioOffset = pio_add_program(PIO_PMS_INSTANCE, &uart_rx_program);

    /* Initialise PIO state machine for RX */
    uart_rx_program_init(PIO_PMS_INSTANCE, PIO_PMS_SM_RX,
                         sPioOffset, PIN_PMS_RX, PMS5003_BAUDRATE);

    sInitOk = true;
    return PMS5003_OK;
}

bool PMS5003_IsInitialised(void)
{
    return sInitOk;
}

Pms5003Result PMS5003_Poll(void)
{
    bool frame_completed = false;

    if (!sInitOk)
    {
        return PMS5003_ERR_NOT_INIT;
    }

    /* Drain all available bytes from PIO RX FIFO */
    while (uart_rx_program_available(PIO_PMS_INSTANCE, PIO_PMS_SM_RX))
    {
        uint8_t byte = uart_rx_program_getc(PIO_PMS_INSTANCE, PIO_PMS_SM_RX);
        FeedByte(byte);

        if (sState == STATE_SYNC1 && sFrameIdx == 0 && sDataValid)
        {
            /* A frame was just completed and validated in FeedByte() */
            frame_completed = true;
        }
    }

    if (frame_completed)
    {
        return PMS5003_OK;
    }

    return sDataValid ? PMS5003_ERR_NO_DATA : PMS5003_ERR_NO_DATA;
}

Pms5003Result PMS5003_GetData(Pms5003Data *data)
{
    if (!sInitOk)
    {
        return PMS5003_ERR_NOT_INIT;
    }
    if (data == NULL)
    {
        return PMS5003_ERR_NO_DATA;
    }
    if (!sDataValid)
    {
        return PMS5003_ERR_NO_DATA;
    }

    *data = sLatest;
    return PMS5003_OK;
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

/**
 * \brief   Feeds one byte into the frame parser state machine.
 * \details Synchronises on 0x42 0x4D header.  On receiving a complete 32-byte
 *          frame, validates checksum and parses data fields.
 */
static void FeedByte(uint8_t byte)
{
    switch (sState)
    {
        case STATE_SYNC1:
            if (byte == PMS5003_START_BYTE_1)
            {
                sFrameBuf[0] = byte;
                sFrameIdx    = 1;
                sState       = STATE_SYNC2;
            }
            break;

        case STATE_SYNC2:
            if (byte == PMS5003_START_BYTE_2)
            {
                sFrameBuf[1] = byte;
                sFrameIdx    = 2;
                sState       = STATE_DATA;
            }
            else if (byte == PMS5003_START_BYTE_1)
            {
                /* Possible re-sync: stay looking for 0x4D */
                sFrameBuf[0] = byte;
                sFrameIdx    = 1;
            }
            else
            {
                /* Bad second byte — restart */
                sState    = STATE_SYNC1;
                sFrameIdx = 0;
            }
            break;

        case STATE_DATA:
            sFrameBuf[sFrameIdx++] = byte;

            if (sFrameIdx >= PMS5003_FRAME_LEN)
            {
                /* Full frame received — validate and parse */
                (void)ValidateAndParse();

                /* Reset for next frame */
                sState    = STATE_SYNC1;
                sFrameIdx = 0;
            }
            break;
    }
}

/**
 * \brief   Validates checksum and frame length, then parses data fields.
 * \return  true if the frame is valid, false otherwise.
 */
static bool ValidateAndParse(void)
{
    uint16_t frame_len;
    uint16_t checksum_rx;
    uint16_t checksum_calc = 0;

    /* Verify frame body length field (bytes 2-3) */
    frame_len = BE16(&sFrameBuf[2]);
    if (frame_len != FRAME_BODY_LEN)
    {
        return false;
    }

    /* Compute checksum over bytes [0..29] */
    for (uint8_t i = 0; i < PMS5003_FRAME_LEN - 2U; i++)
    {
        checksum_calc += sFrameBuf[i];
    }

    /* Compare against received checksum (bytes 30-31, big-endian) */
    checksum_rx = BE16(&sFrameBuf[30]);
    if (checksum_calc != checksum_rx)
    {
        return false;
    }

    /* Parse data — all values are big-endian uint16 */
    sLatest.pm1_0_std  = BE16(&sFrameBuf[4]);
    sLatest.pm2_5_std  = BE16(&sFrameBuf[6]);
    sLatest.pm10_std   = BE16(&sFrameBuf[8]);

    sLatest.pm1_0_atm  = BE16(&sFrameBuf[10]);
    sLatest.pm2_5_atm  = BE16(&sFrameBuf[12]);
    sLatest.pm10_atm   = BE16(&sFrameBuf[14]);

    sLatest.cnt_0_3um  = BE16(&sFrameBuf[16]);
    sLatest.cnt_0_5um  = BE16(&sFrameBuf[18]);
    sLatest.cnt_1_0um  = BE16(&sFrameBuf[20]);
    sLatest.cnt_2_5um  = BE16(&sFrameBuf[22]);
    sLatest.cnt_5_0um  = BE16(&sFrameBuf[24]);
    sLatest.cnt_10um   = BE16(&sFrameBuf[26]);

    sDataValid = true;
    return true;
}

/**
 * \brief   Reads a big-endian uint16 from a byte pointer.
 */
static uint16_t BE16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}
