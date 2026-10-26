/*---------------------------------------------------------------------------/
/  ffconf.h — FatFs R0.15 configuration for CONSCIENTIA CanSat
/
/  Project-specific configuration.  This file is copied over the FatFs
/  template at build time by CMakeLists.txt.
/---------------------------------------------------------------------------*/

#ifndef FFCONF_DEF
#define FFCONF_DEF	80286

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/
#define FF_FS_READONLY	0
#define FF_FS_MINIMIZE	0
#define FF_USE_FIND		0
#define FF_USE_MKFS		1
#define FF_USE_FASTSEEK	0
#define FF_USE_EXPAND	0
#define FF_USE_CHMOD	0
#define FF_USE_LABEL	0
#define FF_USE_FORWARD	0
#define FF_USE_STRFUNC	0
#define FF_PRINT_LLI	0
#define FF_PRINT_FLOAT	0
#define FF_STRF_ENCODE	0

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/
#define FF_CODE_PAGE	437
#define FF_USE_LFN		1       /* LFN with static working buffer        */
#define FF_MAX_LFN		127     /* Shorter than default 255 to save RAM  */
#define FF_LFN_UNICODE	0
#define FF_LFN_BUF		127
#define FF_SFN_BUF		12
#define FF_FS_RPATH		0

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/
#define FF_VOLUMES		1
#define FF_STR_VOLUME_ID	0
#define FF_MULTI_PARTITION	0
#define FF_MIN_SS		512
#define FF_MAX_SS		512
#define FF_LBA64		0
#define FF_MIN_GPT		0x10000000
#define FF_USE_TRIM		0

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/
#define FF_FS_TINY		0       /* Each FIL has its own sector buffer     */
#define FF_FS_EXFAT		0
#define FF_FS_NORTC		1       /* No RTC — use fixed build timestamp    */
#define FF_NORTC_MON	10
#define FF_NORTC_MDAY	26
#define FF_NORTC_YEAR	2026
#define FF_FS_NOFSINFO	0
#define FF_FS_LOCK		0
#define FF_FS_REENTRANT	0

#endif /* FFCONF_DEF */
