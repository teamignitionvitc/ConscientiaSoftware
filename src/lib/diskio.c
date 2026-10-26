/**************************************************************************************************
 * File:        diskio.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       FatFs disk I/O adapter for CONSCIENTIA SD card driver.
 * \details     Bridges ChaN's FatFs diskio interface to our sd_card.c
 *              block-level API (SdCard_ReadBlock / SdCard_WriteBlock).
 **************************************************************************************************/

#include "ff.h"
#include "diskio.h"
#include "drivers/sd_card.h"

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(BYTE pdrv)
{
    (void)pdrv;
    if (!SdCard_IsInitialised())
    {
        return STA_NOINIT;
    }
    return 0;
}

/*-----------------------------------------------------------------------*/
/* Initialise a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(BYTE pdrv)
{
    (void)pdrv;

    if (SdCard_IsInitialised())
    {
        return 0;       /* Already initialised — reuse */
    }

    SdCardResult res = SdCard_Init();
    if (res != SD_OK)
    {
        return STA_NOINIT;
    }
    return 0;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    (void)pdrv;

    for (UINT i = 0; i < count; i++)
    {
        if (SdCard_ReadBlock(sector + i, &buff[i * 512]) != SD_OK)
        {
            return RES_ERROR;
        }
    }
    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    (void)pdrv;

    for (UINT i = 0; i < count; i++)
    {
        if (SdCard_WriteBlock(sector + i, &buff[i * 512]) != SD_OK)
        {
            return RES_ERROR;
        }
    }
    return RES_OK;
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)pdrv;

    switch (cmd)
    {
        case CTRL_SYNC:
            return RES_OK;      /* SPI writes are synchronous */

        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            return RES_OK;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1; /* Unknown erase block size */
            return RES_OK;

        case GET_SECTOR_COUNT:
            /* Required for f_mkfs().  Report ~2 GB (safe for any ≥2 GB card). */
            *(LBA_t *)buff = (LBA_t)(2UL * 1024UL * 1024UL * 1024UL / 512UL);
            return RES_OK;

        default:
            return RES_PARERR;
    }
}
