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
 * File:        sd_card.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Header file for Micro SD card SPI driver.
 * \details     Provides SPI block-level access to an SD/SDHC card over
 *              SPI1 (GPIO10-13) and a raw sequential byte-stream "file"
 *              abstraction suitable for CSV mission logging.
 *
 *              Architecture note:
 *              This driver implements a raw sector I/O layer.  A full
 *              FatFs filesystem can later be layered on top via a diskio.c
 *              adapter that calls SdCard_ReadBlock() / SdCard_WriteBlock().
 *              The byte-stream open/write/flush/close API allows immediate
 *              use for sequential logging without a filesystem dependency.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef SD_CARD_H_
#define SD_CARD_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * Macro Definitions
 **************************************************************************************************/

/** \brief  SD card sector size in bytes (fixed by SD specification). */
#define SD_BLOCK_SIZE           (512U)

/**************************************************************************************************
 * Type Definitions
 **************************************************************************************************/

/**
 * \brief   Result codes returned by SD card driver functions.
 */
typedef enum
{
	SD_OK = 0,              /**< Operation succeeded.                        */
	SD_ERR_INIT,            /**< SPI SD card initialisation failed.          */
	SD_ERR_NOT_INIT,        /**< Driver used before successful init.         */
	SD_ERR_NOT_OPEN,        /**< Write attempted without SdCard_Open().      */
	SD_ERR_READ,            /**< Block read command failed.                  */
	SD_ERR_WRITE,           /**< Block write command failed.                 */
	SD_ERR_TIMEOUT          /**< SD card did not respond within timeout.     */
} SdCardResult;

/**
 * \brief   SD/SDHC card variant detected during init.
 */
typedef enum
{
	SD_TYPE_UNKNOWN = 0,
	SD_TYPE_SDv1,           /**< SD version 1 (SDSC, byte addressing).      */
	SD_TYPE_SDv2,           /**< SD version 2 SDSC (byte addressing).       */
	SD_TYPE_SDHC            /**< SDHC/SDXC (block addressing).              */
} SdCardType;

/**************************************************************************************************
 * Public Function Prototypes — Block Layer
 **************************************************************************************************/

/**
 * \brief   Initialises SPI1 and the SD card SPI protocol.
 * \details Runs the SD SPI power-up sequence (CMD0, CMD8, ACMD41, CMD58),
 *          switches to high-speed SPI, and detects the card type.
 *
 * \return  SD_OK          on success.
 * \return  SD_ERR_INIT    if the card does not respond or init fails.
 * \return  SD_ERR_TIMEOUT if any command times out.
 */
SdCardResult SdCard_Init(void);

/** \brief  Returns true if the card was initialised successfully. */
bool SdCard_IsInitialised(void);

/** \brief  Returns the detected card type. */
SdCardType SdCard_GetType(void);

/**
 * \brief   Reads one 512-byte block from the card.
 *
 * \param[in]  sector   Block address (always in blocks for SDHC, or × 512 for v1/v2).
 * \param[out] buf      Caller buffer of at least SD_BLOCK_SIZE bytes.
 *
 * \return  SD_OK / SD_ERR_NOT_INIT / SD_ERR_READ / SD_ERR_TIMEOUT.
 */
SdCardResult SdCard_ReadBlock(uint32_t sector, uint8_t *buf);

/**
 * \brief   Writes one 512-byte block to the card.
 *
 * \param[in] sector   Block address.
 * \param[in] buf      Data buffer of exactly SD_BLOCK_SIZE bytes.
 *
 * \return  SD_OK / SD_ERR_NOT_INIT / SD_ERR_WRITE / SD_ERR_TIMEOUT.
 */
SdCardResult SdCard_WriteBlock(uint32_t sector, const uint8_t *buf);

/**************************************************************************************************
 * Public Function Prototypes — Sequential Byte-Stream File Layer
 *
 * Provides a simple open/write/flush/close API for sequential byte-stream
 * logging without a filesystem.  Data is written in 512-byte sector chunks
 * starting at a fixed start sector (SD_LOG_START_SECTOR in sd_card.c).
 * A later FatFs module can replace this layer.
 **************************************************************************************************/

/**
 * \brief   Opens the log area for sequential writing from the start sector.
 * \details Resets the internal write pointer and clears the sector buffer.
 *
 * \return  SD_OK / SD_ERR_NOT_INIT.
 */
SdCardResult SdCard_Open(void);

/**
 * \brief   Writes bytes to the sequential log stream.
 * \details Data is buffered internally; full 512-byte sectors are flushed
 *          to the card automatically.  Safe to call with any length.
 *
 * \param[in] data   Pointer to data to write.
 * \param[in] len    Number of bytes.
 *
 * \return  SD_OK / SD_ERR_NOT_INIT / SD_ERR_NOT_OPEN / SD_ERR_WRITE.
 */
SdCardResult SdCard_Write(const uint8_t *data, uint16_t len);

/**
 * \brief   Flushes any buffered partial sector to the card (zero-padded).
 *
 * \return  SD_OK / SD_ERR_NOT_INIT / SD_ERR_NOT_OPEN / SD_ERR_WRITE.
 */
SdCardResult SdCard_Flush(void);

/**
 * \brief   Flushes and closes the current log stream.
 *
 * \return  SD_OK or propagated write error.
 */
SdCardResult SdCard_Close(void);

#endif /* SD_CARD_H_ */
