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
 * File:        bmp280.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Header file for BMP280 barometric pressure / temperature driver.
 * \details     Provides the public API for initializing, configuring, and reading
 *              the Bosch BMP280 sensor over I2C on the Waveshare RP2040-Zero.
 *              This is a hardware driver only — no flight logic or telemetry.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef BMP280_H_
#define BMP280_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * Macro Definitions
 **************************************************************************************************/

/** \brief  Expected chip-ID value read from register 0xD0. */
#define BMP280_CHIP_ID                  (0x58U)

/** \brief  Default sea-level pressure in Pa for altitude estimation.
 *  \details Caller may override at runtime via BMP280_SetSeaLevelPressure().
 */
#define BMP280_DEFAULT_SEA_LEVEL_PA     (101325.0f)

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by BMP280 driver functions.
 */
typedef enum
{
	BMP280_OK = 0,              /**< Operation succeeded.                   */
	BMP280_ERR_I2C,             /**< I2C read/write failure.                */
	BMP280_ERR_CHIP_ID,         /**< Unexpected chip-ID (not 0x58).         */
	BMP280_ERR_CALIB,           /**< Failed to read calibration data.       */
	BMP280_ERR_NOT_INIT         /**< Driver used before successful init.    */
} Bmp280Result;

/**
 * \brief   Compensated sensor readings from the BMP280.
 */
typedef struct
{
	int32_t		temperature_cdegc;  /**< Temperature in 0.01 °C  (5123 = 51.23 °C). */
	uint32_t	pressure_pa;        /**< Pressure in Pa          (96386 = 963.86 hPa). */
} Bmp280Data;

/**************************************************************************************************
 * Public Function Prototypes
 **************************************************************************************************/

/**
 * \brief   Initialises the BMP280 driver.
 * \details Verifies the chip ID, reads factory calibration coefficients, and
 *          writes the initial operating configuration.  The I2C bus must already
 *          be initialised by the caller (pins and baud rate) before calling
 *          this function.
 *
 * \return  BMP280_OK            on success.
 * \return  BMP280_ERR_I2C       if I2C communication fails.
 * \return  BMP280_ERR_CHIP_ID   if the chip-ID register does not read 0x58.
 * \return  BMP280_ERR_CALIB     if calibration data cannot be read.
 */
Bmp280Result BMP280_Init(void);

/**
 * \brief   Checks whether the driver has been initialised successfully.
 *
 * \return  true if BMP280_Init() completed without error, false otherwise.
 */
bool BMP280_IsInitialised(void);

/**
 * \brief   Reads compensated temperature from the BMP280.
 *
 * \param[out] temperature_cdegc   Pointer to receive temperature in 0.01 °C.
 *
 * \return  BMP280_OK            on success.
 * \return  BMP280_ERR_NOT_INIT  if the driver is not initialised.
 * \return  BMP280_ERR_I2C       if I2C communication fails.
 */
Bmp280Result BMP280_ReadTemperature(int32_t *temperature_cdegc);

/**
 * \brief   Reads compensated pressure from the BMP280.
 * \details Temperature compensation is performed internally first because
 *          the pressure formula depends on the t_fine intermediate value.
 *
 * \param[out] pressure_pa   Pointer to receive pressure in Pa.
 *
 * \return  BMP280_OK            on success.
 * \return  BMP280_ERR_NOT_INIT  if the driver is not initialised.
 * \return  BMP280_ERR_I2C       if I2C communication fails.
 */
Bmp280Result BMP280_ReadPressure(uint32_t *pressure_pa);

/**
 * \brief   Reads compensated temperature and pressure in a single burst.
 * \details Both raw values are read from consecutive registers in one I2C
 *          transaction to ensure temporal consistency.
 *
 * \param[out] data   Pointer to Bmp280Data struct to receive both readings.
 *
 * \return  BMP280_OK            on success.
 * \return  BMP280_ERR_NOT_INIT  if the driver is not initialised.
 * \return  BMP280_ERR_I2C       if I2C communication fails.
 */
Bmp280Result BMP280_Read(Bmp280Data *data);

/**
 * \brief   Sets the reference sea-level pressure used for altitude estimation.
 *
 * \param[in] sea_level_pa   Sea-level pressure in Pa (e.g. 101325.0).
 */
void BMP280_SetSeaLevelPressure(float sea_level_pa);

/**
 * \brief   Computes an estimated altitude from the last pressure reading.
 * \details Uses the barometric formula with the configured sea-level
 *          reference pressure.  This is a derived convenience — accuracy
 *          depends on the reference value and prevailing weather conditions.
 *
 * \param[in]  pressure_pa    Measured pressure in Pa.
 * \param[out] altitude_m     Pointer to receive estimated altitude in metres.
 *
 * \return  BMP280_OK            on success.
 * \return  BMP280_ERR_NOT_INIT  if the driver is not initialised.
 */
Bmp280Result BMP280_EstimateAltitude(uint32_t pressure_pa, float *altitude_m);

#endif /* BMP280_H_ */
