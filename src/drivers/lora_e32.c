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
 * File:        lora_e32.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Source file for the LoRa E32-433T3D radio module driver.
 *
 * Description:
 *   Transparent UART mode driver for the EBYTE E32-433T3D LoRa module
 *   over Pico SDK hardware UART0 (GPIO0 TX, GPIO1 RX, 9600 baud).
 *   Control pins: M0 (GPIO2), M1 (GPIO3) as outputs, AUX (GPIO6) as input.
 *
 *   E32 mode selection (M0, M1):
 *     Normal (0,0) — transparent UART pass-through (primary flight mode).
 *     Wakeup (1,0) — sends wake-up preamble before each message.
 *     PowerSave (0,1) — listen-before-talk, periodic wake.
 *     Sleep  (1,1) — configuration / sleep mode.
 *
 *   AUX pin:
 *     HIGH = module ready / idle.
 *     LOW  = module busy (sending, receiving, or changing mode).
 *     All blocking waits use LoraE32_WaitAuxReady() with a bounded timeout.
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
#include "lora_e32.h"
#include "config.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/

/** Post-mode-switch stabilisation delay before checking AUX (ms). */
#define MODE_SWITCH_DELAY_MS    (10U)

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool sInitOk = false;

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/
static void ApplyMode(LoraE32Mode mode);

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

LoraE32Result LoraE32_Init(void)
{
    LoraE32Result res;

    sInitOk = false;

    /* Configure UART0 */
    uart_init(UART0_INSTANCE, UART0_BAUDRATE);
    gpio_set_function(PIN_UART0_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART0_RX, GPIO_FUNC_UART);

    /* Configure M0, M1 as outputs; AUX as input with pull-down */
    gpio_init(PIN_LORA_M0);
    gpio_init(PIN_LORA_M1);
    gpio_init(PIN_LORA_AUX);

    gpio_set_dir(PIN_LORA_M0,  GPIO_OUT);
    gpio_set_dir(PIN_LORA_M1,  GPIO_OUT);
    gpio_set_dir(PIN_LORA_AUX, GPIO_IN);
    gpio_pull_down(PIN_LORA_AUX);

    /* Place module in NORMAL transparent mode */
    ApplyMode(LORA_MODE_NORMAL);
    sleep_ms(MODE_SWITCH_DELAY_MS);

    /* Wait for AUX to confirm readiness */
    res = LoraE32_WaitAuxReady(LORA_E32_AUX_TIMEOUT_MS);
    if (res != LORA_OK)
    {
        return res;
    }

    sInitOk = true;
    return LORA_OK;
}

bool LoraE32_IsInitialised(void)
{
    return sInitOk;
}

LoraE32Result LoraE32_Send(const uint8_t *data, uint8_t len)
{
    if (!sInitOk)
    {
        return LORA_ERR_NOT_INIT;
    }
    if (data == NULL || len == 0U)
    {
        return LORA_OK;
    }

    uart_write_blocking(UART0_INSTANCE, data, len);
    return LORA_OK;
}

LoraE32Result LoraE32_Receive(uint8_t *buf, uint8_t buf_len,
                               uint8_t *bytes_read)
{
    uint8_t count = 0;
    bool overflow = false;

    if (!sInitOk)
    {
        return LORA_ERR_NOT_INIT;
    }
    if (buf == NULL || bytes_read == NULL)
    {
        return LORA_ERR_NOT_INIT;
    }

    while (uart_is_readable(UART0_INSTANCE))
    {
        uint8_t byte = (uint8_t)uart_getc(UART0_INSTANCE);

        if (count < buf_len)
        {
            buf[count] = byte;
            count++;
        }
        else
        {
            overflow = true;
            /* Drain the remaining byte to clear the FIFO — discard it */
        }
    }

    *bytes_read = count;
    return overflow ? LORA_ERR_OVERFLOW : LORA_OK;
}

bool LoraE32_IsAuxReady(void)
{
    return gpio_get(PIN_LORA_AUX);
}

LoraE32Result LoraE32_WaitAuxReady(uint32_t timeout_ms)
{
    uint32_t elapsed = 0U;
    const uint32_t POLL_MS = 2U;

    while (!LoraE32_IsAuxReady())
    {
        if (elapsed >= timeout_ms)
        {
            return LORA_ERR_AUX_TIMEOUT;
        }
        sleep_ms(POLL_MS);
        elapsed += POLL_MS;
    }

    return LORA_OK;
}

LoraE32Result LoraE32_SetMode(LoraE32Mode mode)
{
    LoraE32Result res;

    if (!sInitOk)
    {
        return LORA_ERR_NOT_INIT;
    }

    ApplyMode(mode);
    sleep_ms(MODE_SWITCH_DELAY_MS);

    res = LoraE32_WaitAuxReady(LORA_E32_AUX_TIMEOUT_MS);
    return res;
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

/**
 * \brief   Drives M0/M1 GPIO pins to select the E32 operating mode.
 */
static void ApplyMode(LoraE32Mode mode)
{
    gpio_put(PIN_LORA_M0, (mode & 0x01U) ? 1 : 0);
    gpio_put(PIN_LORA_M1, (mode & 0x02U) ? 1 : 0);
}
