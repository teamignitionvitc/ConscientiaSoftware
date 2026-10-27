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
 * File:        bh1750.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Source file for the BH1750 ambient light sensor driver (instance-based).
 *
 * Description:
 *   Implements I2C communication for the ROHM BH1750FVI ambient light sensor
 *   on the Waveshare RP2040-Zero using the Pico SDK hardware I2C API.
 *
 *   The driver is instance-based: each physical sensor is represented by a
 *   Bh1750Dev descriptor that stores its I2C bus, address, mode, MTreg, and
 *   initialisation state.  This allows three independent BH1750 sensors on
 *   CONSCIENTIA (two on I2C0 at 0x23 / 0x5C, one on I2C1 at 0x23) without
 *   code duplication.
 *
 *   Default configuration on init:
 *     Power On   -> 0x01
 *     Reset      -> 0x07
 *     MTreg      -> 69  (factory default)
 *     Mode       -> Continuous High Resolution (0x10) — 1 lx, ~120 ms
 *
 *   Lux conversion formula (from datasheet):
 *     Mode 1 / L-Res:  lux = raw / 1.2 * (69 / mtreg)
 *     Mode 2:          lux = raw / 1.2 * (69 / mtreg) / 2
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
#include "bh1750.h"
#include "config.h"

#include "pico/stdlib.h"

/**************************************************************************************************
 * BH1750 Instruction Opcodes
 **************************************************************************************************/
#define CMD_POWER_DOWN      (0x00U)
#define CMD_POWER_ON        (0x01U)
#define CMD_RESET           (0x07U)

/** High-byte MTreg instruction: 01000_MT[7:5]. */
#define CMD_MTREG_HI(mt)   (0x40U | (uint8_t)((mt) >> 5))

/** Low-byte MTreg instruction: 011_MT[4:0]. */
#define CMD_MTREG_LO(mt)   (0x60U | (uint8_t)((mt) & 0x1FU))

/**************************************************************************************************
 * Internal Constants
 **************************************************************************************************/

/** I2C timeout per transaction (us). */
#define I2C_TIMEOUT_US      (10000)

/**************************************************************************************************
 * Static Function Declarations
 **************************************************************************************************/
static Bh1750Result SendCommand(const Bh1750Dev *dev, uint8_t cmd);
static bool         ModeIsValid(Bh1750Mode mode);

/**************************************************************************************************
 * Function Definitions
 **************************************************************************************************/

Bh1750Result BH1750_Init(Bh1750Dev *dev)
{
    Bh1750Result res;

    if (dev == NULL)
    {
        return BH1750_ERR_PARAM;
    }

    dev->init_ok = false;

    /* Power on the sensor */
    res = SendCommand(dev, CMD_POWER_ON);
    if (res != BH1750_OK)
    {
        return res;
    }

    /* Reset data register (must be in powered-on state) */
    res = SendCommand(dev, CMD_RESET);
    if (res != BH1750_OK)
    {
        return res;
    }

    /* Write default MTreg */
    dev->mtreg = BH1750_DEFAULT_MTREG;

    res = SendCommand(dev, CMD_MTREG_HI(BH1750_DEFAULT_MTREG));
    if (res != BH1750_OK)
    {
        return res;
    }

    res = SendCommand(dev, CMD_MTREG_LO(BH1750_DEFAULT_MTREG));
    if (res != BH1750_OK)
    {
        return res;
    }

    /* Start continuous high-resolution measurement (default mode) */
    dev->mode = BH1750_MODE_CONT_HRES;

    res = SendCommand(dev, (uint8_t)dev->mode);
    if (res != BH1750_OK)
    {
        return res;
    }

    dev->init_ok = true;
    return BH1750_OK;
}

bool BH1750_IsInitialised(const Bh1750Dev *dev)
{
    if (dev == NULL)
    {
        return false;
    }
    return dev->init_ok;
}

Bh1750Result BH1750_SetMode(Bh1750Dev *dev, Bh1750Mode mode)
{
    Bh1750Result res;

    if (dev == NULL)
    {
        return BH1750_ERR_PARAM;
    }
    if (!dev->init_ok)
    {
        return BH1750_ERR_NOT_INIT;
    }
    if (!ModeIsValid(mode))
    {
        return BH1750_ERR_PARAM;
    }

    res = SendCommand(dev, (uint8_t)mode);
    if (res != BH1750_OK)
    {
        return res;
    }

    dev->mode = mode;
    return BH1750_OK;
}

Bh1750Result BH1750_SetMTreg(Bh1750Dev *dev, uint8_t mtreg)
{
    Bh1750Result res;

    if (dev == NULL)
    {
        return BH1750_ERR_PARAM;
    }
    if (!dev->init_ok)
    {
        return BH1750_ERR_NOT_INIT;
    }
    if (mtreg < BH1750_MTREG_MIN || mtreg > BH1750_MTREG_MAX)
    {
        return BH1750_ERR_PARAM;
    }

    /* Program MTreg via high-byte then low-byte commands */
    res = SendCommand(dev, CMD_MTREG_HI(mtreg));
    if (res != BH1750_OK)
    {
        return res;
    }

    res = SendCommand(dev, CMD_MTREG_LO(mtreg));
    if (res != BH1750_OK)
    {
        return res;
    }

    dev->mtreg = mtreg;

    /* Re-start current continuous mode to apply new MTreg */
    res = SendCommand(dev, (uint8_t)dev->mode);
    if (res != BH1750_OK)
    {
        return res;
    }

    return BH1750_OK;
}

Bh1750Result BH1750_ReadRaw(const Bh1750Dev *dev, uint16_t *raw_out)
{
    uint8_t buf[2];
    int ret;

    if (dev == NULL || raw_out == NULL)
    {
        return BH1750_ERR_PARAM;
    }
    if (!dev->init_ok)
    {
        return BH1750_ERR_NOT_INIT;
    }

    /* BH1750 returns 2 bytes (MSB first) on a read without register address */
    ret = i2c_read_timeout_us(dev->i2c, dev->addr,
                              buf, 2, false, I2C_TIMEOUT_US);
    if (ret != 2)
    {
        return BH1750_ERR_I2C;
    }

    *raw_out = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return BH1750_OK;
}

Bh1750Result BH1750_ReadLux(const Bh1750Dev *dev, float *lux_out)
{
    uint16_t raw;
    Bh1750Result res;

    if (lux_out == NULL)
    {
        return BH1750_ERR_PARAM;
    }

    res = BH1750_ReadRaw(dev, &raw);
    if (res != BH1750_OK)
    {
        return res;
    }

    /*
     * Datasheet lux formula:
     *   lux = raw / 1.2 * (69 / mtreg)            [H-Res Mode 1 & L-Res]
     *   lux = raw / 1.2 * (69 / mtreg) / 2         [H-Res Mode 2]
     *
     * Rearranged to minimise floating-point operations:
     *   scale = (69.0 / mtreg) / 1.2
     *   if Mode 2: scale /= 2.0
     *   lux = raw * scale
     */
    float scale = 69.0f / ((float)dev->mtreg * 1.2f);

    if (dev->mode == BH1750_MODE_CONT_HRES2)
    {
        scale *= 0.5f;
    }

    *lux_out = (float)raw * scale;
    return BH1750_OK;
}

/**************************************************************************************************
 * Static Function Definitions
 **************************************************************************************************/

/**
 * \brief   Sends a single-byte command/opcode to the BH1750 over I2C.
 */
static Bh1750Result SendCommand(const Bh1750Dev *dev, uint8_t cmd)
{
    int ret = i2c_write_timeout_us(dev->i2c, dev->addr,
                                   &cmd, 1, false, I2C_TIMEOUT_US);
    return (ret == 1) ? BH1750_OK : BH1750_ERR_I2C;
}

/**
 * \brief   Validates a Bh1750Mode enum value.
 */
static bool ModeIsValid(Bh1750Mode mode)
{
    return (mode == BH1750_MODE_CONT_HRES  ||
            mode == BH1750_MODE_CONT_HRES2 ||
            mode == BH1750_MODE_CONT_LRES);
}
