#ifndef MX_FLASH_FS_H
#define	MX_FLASH_FS_H

	#include <stdint.h>
	#include <stdbool.h>
    #include "ff.h"
    #include "mx_flash_fs.h"
    #include "integer.h"

    #define MX_FLASH_DEBUG  0

    bool    fatfs_init          (void);
    void    MXflash_FSinit      (void);
    void    MXflash_FSdeinit    (void);
    UINT    fatfs_write         (char * dataToWrite, UINT lenghtToWrite);
    void    fatfs_read          (void);
    FRESULT fatfs_clear         (void);
    void    fatfs_clearAll      (void);

    FRESULT fatfs_open          (FIL * fp, char * fname, FSIZE_t offset);
    FRESULT fatfs_close         (FIL * fp);
    FRESULT fatfs_readBuffer    (FIL * fp, char * readBuff, UINT size, UINT * bytesRead);

#endif	/* MX_FLASH_FS_H */

