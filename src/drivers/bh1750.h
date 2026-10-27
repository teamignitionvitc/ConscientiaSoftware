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
 * File:        bh1750.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Header file for BH1750 ambient light sensor driver (instance-based).
 * \details     Provides a reusable, instance-based API for the ROHM BH1750FVI
 *              digital ambient light sensor over I2C.  Supports multiple independent
 *              sensors on different bus/address combinations (e.g. three BH1750s on
 *              CONSCIENTIA).  This is a hardware driver only — no fusion, payload
 *              processing, or telemetry logic.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef BH1750_H_
#define BH1750_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

/**************************************************************************************************
 * Macro Definitions
 **************************************************************************************************/

/** \brief  Default measurement-time register value (factory default). */
#define BH1750_DEFAULT_MTREG            (69U)

/** \brief  Minimum allowable MTreg value. */
#define BH1750_MTREG_MIN                (31U)

/** \brief  Maximum allowable MTreg value. */
#define BH1750_MTREG_MAX                (254U)

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by BH1750 driver functions.
 */
typedef enum
{
	BH1750_OK = 0,              /**< Operation succeeded.                   */
	BH1750_ERR_I2C,             /**< I2C read/write failure.                */
	BH1750_ERR_NOT_INIT,        /**< Driver used before successful init.    */
	BH1750_ERR_PARAM            /**< Invalid parameter supplied.            */
} Bh1750Result;

/**
 * \brief   Continuous measurement modes supported by the BH1750.
 */
typedef enum
{
	BH1750_MODE_CONT_HRES  = 0x10U,  /**< Continuous High Resolution — 1 lx, ~120 ms.   */
	BH1750_MODE_CONT_HRES2 = 0x11U,  /**< Continuous High Resolution 2 — 0.5 lx, ~120 ms. */
	BH1750_MODE_CONT_LRES  = 0x13U   /**< Continuous Low Resolution  — 4 lx, ~16 ms.    */
} Bh1750Mode;

/**
 * \brief   Instance descriptor for one BH1750 sensor.
 * \details The caller allocates this struct (typically as a static variable)
 *          and passes it to every BH1750 function.  Each descriptor stores its
 *          own bus, address, mode, MTreg, and initialisation state.
 */
typedef struct
{
	i2c_inst_t *i2c;            /**< Pico SDK I2C instance (i2c0 / i2c1).   */
	uint8_t     addr;           /**< 7-bit I2C slave address.               */
	Bh1750Mode  mode;           /**< Active measurement mode.               */
	uint8_t     mtreg;          /**< Current measurement-time register.     */
	bool        init_ok;        /**< true after successful init.            */
} Bh1750Dev;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises a BH1750 sensor instance.
 * \details Powers on the device, resets the data register, writes the
 *          default MTreg (69), and starts continuous high-resolution
 *          measurement.  The I2C bus must already be initialised by the
 *          caller before calling this function.
 *
 * \param[in,out] dev    Pointer to the device descriptor.  The caller must
 *                       pre-fill the \c i2c and \c addr fields before calling.
 *
 * \return  BH1750_OK          on success.
 * \return  BH1750_ERR_PARAM   if \p dev is NULL.
 * \return  BH1750_ERR_I2C     if I2C communication fails.
 */
Bh1750Result BH1750_Init(Bh1750Dev *dev);

/**
 * \brief   Checks whether a BH1750 instance has been initialised.
 *
 * \param[in] dev   Pointer to the device descriptor.
 *
 * \return  true if BH1750_Init() completed without error, false otherwise.
 */
bool BH1750_IsInitialised(const Bh1750Dev *dev);

/**
 * \brief   Changes the continuous measurement mode.
 * \details Sends the new mode opcode.  The sensor begins measuring in the
 *          new mode automatically.
 *
 * \param[in,out] dev    Pointer to the device descriptor.
 * \param[in]     mode   Desired continuous measurement mode.
 *
 * \return  BH1750_OK            on success.
 * \return  BH1750_ERR_NOT_INIT  if the device is not initialised.
 * \return  BH1750_ERR_PARAM     if the mode value is invalid.
 * \return  BH1750_ERR_I2C       if I2C communication fails.
 */
Bh1750Result BH1750_SetMode(Bh1750Dev *dev, Bh1750Mode mode);

/**
 * \brief   Sets the measurement-time register (MTreg) to adjust sensitivity.
 * \details A higher MTreg increases integration time and range in low-light
 *          conditions.  After writing MTreg the sensor is re-started in its
 *          current continuous mode.
 *
 * \param[in,out] dev      Pointer to the device descriptor.
 * \param[in]     mtreg    New MTreg value (31..254).
 *
 * \return  BH1750_OK            on success.
 * \return  BH1750_ERR_NOT_INIT  if the device is not initialised.
 * \return  BH1750_ERR_PARAM     if mtreg is out of range.
 * \return  BH1750_ERR_I2C       if I2C communication fails.
 */
Bh1750Result BH1750_SetMTreg(Bh1750Dev *dev, uint8_t mtreg);

/**
 * \brief   Reads the raw 16-bit measurement from the sensor.
 *
 * \param[in]  dev        Pointer to the device descriptor.
 * \param[out] raw_out    Pointer to receive the raw 16-bit count.
 *
 * \return  BH1750_OK            on success.
 * \return  BH1750_ERR_NOT_INIT  if the device is not initialised.
 * \return  BH1750_ERR_I2C       if I2C communication fails.
 */
Bh1750Result BH1750_ReadRaw(const Bh1750Dev *dev, uint16_t *raw_out);

/**
 * \brief   Reads the ambient light level in lux.
 * \details The raw count is converted to lux accounting for the current
 *          measurement mode and MTreg setting:
 *
 *            lux = raw / 1.2 * (69 / mtreg)           [Mode 1 & L-Res]
 *            lux = raw / 1.2 * (69 / mtreg) / 2       [Mode 2]
 *
 * \param[in]  dev       Pointer to the device descriptor.
 * \param[out] lux_out   Pointer to receive illuminance in lux.
 *
 * \return  BH1750_OK            on success.
 * \return  BH1750_ERR_NOT_INIT  if the device is not initialised.
 * \return  BH1750_ERR_I2C       if I2C communication fails.
 */
Bh1750Result BH1750_ReadLux(const Bh1750Dev *dev, float *lux_out);

#endif /* BH1750_H_ */
