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
                        .:::::::::::::::::::::::::::::::::.        

                    © Copyright of Ignition Avionics Department
*/

/**************************************************************************************************
 * File:        servo.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Source file for the MG90S servo PWM driver.
 *
 * Description:
 *   Generates a 50 Hz servo-control PWM signal on GPIO7 using the
 *   RP2040 hardware PWM peripheral.
 *
 *   PWM configuration:
 *     sys_clk = configured clock (125 or 133 MHz)
 *     clkdiv  = sys_clk_hz / 1 MHz  (computed at runtime)
 *     wrap    = 20000 → period = 20 ms = 50 Hz
 *     level (compare) = pulse_us, directly in µs
 *
 *   Pulse limits (from config.h):
 *     SERVO_PULSE_MIN_US     = 500   (stow)
 *     SERVO_PULSE_MAX_US     = 2400  (deploy)
 *     SERVO_PULSE_NEUTRAL_US = 1450  (neutral/centre)
 *
 *   All inputs are clamped to [MIN, MAX] before writing the comparator.
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
#include "servo.h"
#include "config.h"

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/

/**
 * PWM wrap value: 20000 ticks = 20 ms period = 50 Hz.
 * Clock divider is computed at runtime so that 1 tick = 1 µs regardless
 * of the configured sys_clk (125 or 133 MHz).
 */
#define PWM_WRAP        (20000U)

/**************************************************************************************************
 * Static Variables
 **************************************************************************************************/
static bool     sInitOk  = false;
static uint     sSlice   = 0;
static uint     sChannel = 0;

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/
static uint16_t ClampPulse(uint16_t pulse_us);

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

ServoResult Servo_Init(void)
{
    sInitOk = false;

    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);

    sSlice   = pwm_gpio_to_slice_num(PIN_SERVO);
    sChannel = pwm_gpio_to_channel(PIN_SERVO);

    /* Compute integer divider so 1 tick = 1 µs at the actual sys_clk.
     * e.g. 133 MHz → divider = 133, 125 MHz → divider = 125.         */
    uint32_t sys_hz = clock_get_hz(clk_sys);
    uint16_t clkdiv = (uint16_t)(sys_hz / 1000000U);
    if (clkdiv == 0) { clkdiv = 125U; }   /* fallback */

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, clkdiv);
    pwm_config_set_wrap(&cfg, PWM_WRAP - 1U);

    pwm_init(sSlice, &cfg, true);          /* Start PWM immediately */
    pwm_set_chan_level(sSlice, sChannel, SERVO_PULSE_NEUTRAL_US);

    sInitOk = true;
    return SERVO_OK;
}

bool Servo_IsInitialised(void)
{
    return sInitOk;
}

ServoResult Servo_SetPulse(uint16_t pulse_us)
{
    if (!sInitOk)
    {
        return SERVO_ERR_NOT_INIT;
    }

    pwm_set_chan_level(sSlice, sChannel, ClampPulse(pulse_us));
    return SERVO_OK;
}

ServoResult Servo_SetAngle(uint8_t angle_deg)
{
    uint32_t range;
    uint32_t pulse_us;

    if (!sInitOk)
    {
        return SERVO_ERR_NOT_INIT;
    }

    if (angle_deg > 180U)
    {
        angle_deg = 180U;
    }

    range    = SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US;
    pulse_us = SERVO_PULSE_MIN_US + (range * (uint32_t)angle_deg) / 180U;

    pwm_set_chan_level(sSlice, sChannel, ClampPulse((uint16_t)pulse_us));
    return SERVO_OK;
}

ServoResult Servo_Stow(void)
{
    return Servo_SetPulse(SERVO_PULSE_MIN_US);
}

ServoResult Servo_Deploy(void)
{
    return Servo_SetPulse(SERVO_PULSE_MAX_US);
}

ServoResult Servo_Neutral(void)
{
    return Servo_SetPulse(SERVO_PULSE_NEUTRAL_US);
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

static uint16_t ClampPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_PULSE_MIN_US) { return SERVO_PULSE_MIN_US; }
    if (pulse_us > SERVO_PULSE_MAX_US) { return SERVO_PULSE_MAX_US; }
    return pulse_us;
}
