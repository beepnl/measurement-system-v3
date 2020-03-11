#define NRF_LOG_MODULE_NAME MX_FS
#include "mx_flash_fs.h"
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf.h"
#include "nrf_block_dev.h"
#include "nrf_block_dev_ram.h"
#include "nrf_block_dev_empty.h"
#include "nrf_block_dev_qspi.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_atomic.h"
#include "nrf_drv_power.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "app_timer.h"
#include "gpio-board.h"


const char filename[] = {"BEEPlog.txt"};

/**
 * @brief  QSPI block device definition
 */
NRF_BLOCK_DEV_QSPI_DEFINE(
    m_block_dev_qspi,
    NRF_BLOCK_DEV_QSPI_CONFIG(
        512,
        NRF_BLOCK_DEV_QSPI_FLAG_CACHE_WRITEBACK,
        NRF_DRV_QSPI_DEFAULT_CONFIG
     ),
     NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "QSPI", "1.00")
);


#define BLOCKDEV_LIST() (                                   \
    NRF_BLOCKDEV_BASE_ADDR(m_block_dev_ram, block_dev),     \
    NRF_BLOCKDEV_BASE_ADDR(m_block_dev_empty, block_dev),   \
    NRF_BLOCKDEV_BASE_ADDR(m_block_dev_qspi, block_dev)     \
)


static FATFS m_filesystem;



static bool fatfs_mkfs(void)
{
    FRESULT ff_result;

    
    static uint8_t buf[512];
    ff_result = f_mkfs("", FM_FAT, 1024, buf, sizeof(buf));

    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Creating filesystem...%s", (ff_result == FR_OK) ? "succeeded" : "failed");
    #endif

    if (ff_result != FR_OK)
    {
        return false;
    }

    ff_result = f_mount(&m_filesystem, "", 1);
        
    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Mounting volume...%s", (ff_result == FR_OK) ? "succeeded" : "failed");
    #endif
    if (ff_result != FR_OK)
    {
        return false;
    }
    return true;
}

static void fatfs_uninit(void)
{
    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Un-initializing disk 0 (QSPI)...");
    #endif
    UNUSED_RETURN_VALUE(disk_uninitialize(0));
}


bool fatfs_init(void)
{
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    memset(&m_filesystem, 0, sizeof(FATFS));

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
        DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_qspi, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    #if MX_FLASH_DEBUG
    NRF_LOG_INFO("Initializing disk 0 (QSPI)...");
    #endif
    disk_state = disk_initialize(0);
    if (disk_state)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_ERROR("Disk initialization failed.");
        #endif
        return false;
    }

    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Mounting volume...");
    #endif

    ff_result = f_mount(&m_filesystem, "", 1);
    if (ff_result != FR_OK)
    {
        if (ff_result == FR_NO_FILESYSTEM)
        {
            #if MX_FLASH_DEBUG
                NRF_LOG_ERROR("Mount failed. Filesystem not found. Please format device.");
            #endif

            // Create  a new file system.    
            fatfs_mkfs();
        }
        else
        {
            #if MX_FLASH_DEBUG
                NRF_LOG_ERROR("Mount failed: %u", ff_result);
            #endif
        }
        return false;
    }
    
    return true;
}




FRESULT fatfs_open(FIL * fp, char * fname, FSIZE_t offset)
{
    DIR dir;
    FRESULT ff_result;
    FILINFO stat;
    
    UINT    bytesRead = 0;

    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Listing directory: /");
    #endif
    ff_result = f_opendir(&dir, "/");
    if (ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_ERROR("Directory listing failed: %u", ff_result);
        #endif
        return ff_result;
    }

    // Open the file
    ff_result = f_open(fp, fname, FA_READ);
    if(ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_INFO("Opening the file didn't succeed: %u", ff_result);
        #endif
        return ff_result;
    }  
         
    ff_result = f_stat(fname, &stat);
    if(ff_result != FR_OK)
    {
        return ff_result;
    }

    // Check if the offsetis larger or equal to the file size.
    if(offset >= stat.fsize)
    {
        #if 1 // MX_FLASH_DEBUG
            NRF_LOG_INFO("Invalid offset: %u", ff_result);
        #endif

        #if 1
            offset = 0; 
        #else   
            return FR_INVALID_PARAMETER;
        #endif
    }
    
    ff_result = f_lseek(fp, offset);
    return ff_result;
}



FRESULT fatfs_close(FIL * fp)
{
    FRESULT ff_result;
    
    ff_result = f_close(fp);	/* close the file */
    if(ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_INFO("closing the file didn't succeed: %u", ff_result);
        #endif
    }
    return ff_result;
}


FRESULT fatfs_readBuffer(FIL * fp, char * readBuff, UINT size, UINT * bytesRead)
{
    FRESULT ff_result;
 
    //  Read data from the file
    ff_result = f_read(fp, (void*) readBuff, size, bytesRead);
    if(ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_INFO("Opening the file didn't succeed: %u", ff_result);
        #endif
    }
    else
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_INFO("Bytes Read: %u", *bytesRead);
            NRF_LOG_HEXDUMP_INFO(readBuff, *bytesRead);
            NRF_LOG_FLUSH();
        #endif
    }
    return ff_result;
}


void fatfs_read(void)
{
    DIR dir;
    FRESULT ff_result;
    FIL fp;
    UINT    bytesRead = 0;

    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Listing directory: /");
    #endif
    ff_result = f_opendir(&dir, "/");
    if (ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_ERROR("Directory listing failed: %u", ff_result);
        #endif
        return;
    }

    // Open the file
    ff_result = f_open(&fp, filename, FA_READ);
    if(ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_INFO("Opening the file didn't succeed: %u", ff_result);
        #endif
        return;
    }


    char readBuff[100] = {0};
    // FRESULT f_read (FIL* fp, void* buff, UINT btr, UINT* br);			/* Read data from the file */
    do
    {
        ff_result = f_read(&fp, (void*) readBuff, sizeof(readBuff), &bytesRead);
        if(ff_result != FR_OK)
        {
            #if MX_FLASH_DEBUG
                NRF_LOG_INFO("Opening the file didn't succeed: %u", ff_result);
            #endif
            return;
        }
        else
        {
            #if MX_FLASH_DEBUG
                NRF_LOG_INFO("Bytes Read: %u", bytesRead);
                NRF_LOG_HEXDUMP_INFO(readBuff, bytesRead);
                NRF_LOG_FLUSH();
            #endif
        }
    // Continue to read the file contents until all data has been read
    }while(bytesRead != 0 && (100 == bytesRead));


    ff_result = f_close(&fp);	/* close the file */
    if(ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_INFO("closing the file didn't succeed: %u", ff_result);
        #endif
        return;
    }
}



UINT fatfs_write(char * dataToWrite, UINT lenghtToWrite)
{
    DIR dir;
    FRESULT ff_result;
    FIL fp;
    UINT    bytesWritten = 0;

    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Listing directory: /");
    #endif
    ff_result = f_opendir(&dir, "/");
    if (ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_ERROR("Directory listing failed: %u", ff_result);
        #endif
        return 0;
    }


    ff_result = f_open(&fp, filename, FA_OPEN_APPEND | FA_WRITE | FA_READ);	/* Open or create a file */
    if(ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_INFO("Opening the file didn't succeed: %u", ff_result);
        #endif
        return 0;
    }

    ff_result = f_write(&fp, (const void*) dataToWrite, lenghtToWrite, &bytesWritten);
    if(ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_INFO("Opening the file didn't succeed: %u", ff_result);
        #endif
        return 0;
    }
    else
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_INFO("Bytes Written: %u", bytesWritten);
        #endif
    }


    ff_result = f_close(&fp);	/* close the file */
    if(ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_INFO("closing the file didn't succeed: %u", ff_result);
        #endif
        return 0;
    }
    return bytesWritten;
}



static void fatfs_ls(void)
{
    DIR dir;
    FRESULT ff_result;
    FILINFO fno;
    uint16_t dirCount = 0;

    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Listing directory: /");
    #endif
    ff_result = f_opendir(&dir, "/");
    if (ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_ERROR("Directory listing failed: %u", ff_result);
        #endif
        return;
    }

    uint32_t entries_count = 0;
    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            #if MX_FLASH_DEBUG
                NRF_LOG_ERROR("Directory read failed: %u", ff_result);
            #endif
            return;
        }

        if (fno.fname[0])
        {
            #if MX_FLASH_DEBUG
                if (fno.fattrib & AM_DIR)
                {
                    NRF_LOG_INFO("%u   <DIR>   %s", dirCount++, (uint32_t)fno.fname);
                }
                else
                {
                    NRF_LOG_INFO("%s, %9lu bytes", (uint32_t)fno.fname, fno.fsize);
                }
            #endif
        }

        ++entries_count;
        NRF_LOG_FLUSH();
    } while (fno.fname[0]);

    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Entries count: %u", entries_count);
        NRF_LOG_FLUSH();
    #endif
}

FRESULT delete_node (
    TCHAR* path,    /* Path name buffer with the sub-directory to delete */
    UINT sz_buff,   /* Size of path name buffer (items) */
    FILINFO* fno    /* Name read buffer */
)
{
    UINT i, j;
    FRESULT fr;
    DIR dir;


    fr = f_opendir(&dir, path); /* Open the directory */
    if (fr != FR_OK) return fr;

    for (i = 0; path[i]; i++) ; /* Get current path length */
    path[i++] = _T('/');

    for (;;) {
        fr = f_readdir(&dir, fno);  /* Get a directory item */
        if (fr != FR_OK || !fno->fname[0]) break;   /* End of directory? */
        j = 0;
        do {    /* Make a path name */
            if (i + j >= sz_buff) { /* Buffer over flow? */
                fr = 100; break;    /* Fails with 100 when buffer overflow */
            }
            path[i + j] = fno->fname[j];
        } while (fno->fname[j++]);
        if (fno->fattrib & AM_DIR) {    /* Item is a directory */
            fr = delete_node(path, sz_buff, fno);
        } else {                        /* Item is a file */
            fr = f_unlink(path);
        }
        if (fr != FR_OK) break;
    }

    path[--i] = 0;  /* Restore the path name */
    f_closedir(&dir);

    if (fr == FR_OK) fr = f_unlink(path);  /* Delete the empty directory */
    return fr;
}


void fatfs_clearAll(void)
{
    DIR dir;
    FRESULT ff_result;
    FILINFO fno;
    uint16_t dirCount = 0;

    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Listing directory: /");
    #endif
    ff_result = f_opendir(&dir, "/");
    if (ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_ERROR("Directory listing failed: %u", ff_result);
        #endif
        return;
    }

    uint32_t entries_count = 0;
    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            #if MX_FLASH_DEBUG
                NRF_LOG_ERROR("Directory read failed: %u", ff_result);
            #endif
            return;
        }

        if (fno.fname[0])
        {
            #if MX_FLASH_DEBUG
                if (fno.fattrib & AM_DIR)
                {
                    ff_result = f_unlink(fno.fname);
                    NRF_LOG_INFO("%u   <DIR>   %s, unlink result: %u", dirCount++, (uint32_t)fno.fname, ff_result);
                }
                else
                {
                    ff_result = f_unlink(fno.fname);
                    NRF_LOG_INFO("%s, %9lu bytes, unlink result: %u", (uint32_t)fno.fname, fno.fsize, ff_result);
                }
            #endif
        }

        ++entries_count;
        NRF_LOG_FLUSH();
    } while (fno.fname[0]);

    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Entries count: %u", entries_count);
        NRF_LOG_FLUSH();
    #endif
}



FRESULT fatfs_clear(void)
{
    DIR dir;
    FRESULT ff_result;
    FILINFO fno;

    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("Listing directory: /");
    #endif

    fatfs_init();

    ff_result = f_opendir(&dir, "/");
    if (ff_result != FR_OK)
    {
        #if MX_FLASH_DEBUG
            NRF_LOG_ERROR("Directory listing failed: %u", ff_result);
        #endif
        MXflash_FSdeinit();
        return ff_result;
    }

    uint32_t entries_count = 0;
    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            #if MX_FLASH_DEBUG
                NRF_LOG_ERROR("Directory read failed: %u", ff_result);
            #endif
            MXflash_FSdeinit();
            return ff_result;
        }

        if (fno.fname[0] != NULL)
        {
            #if MX_FLASH_DEBUG
            if (fno.fattrib & AM_DIR)
            {
                NRF_LOG_INFO("   <DIR>   %s",(uint32_t)fno.fname);
            }
            else
            {
                NRF_LOG_INFO("%9lu  %s", fno.fsize, (uint32_t)fno.fname);
            }
            #endif

            ff_result = f_unlink(fno.fname);
            #if MX_FLASH_DEBUG
                NRF_LOG_ERROR("Unlinking file %s status: %u, %s", fno.fname, ff_result, (ff_result == FR_OK) ? "succeeded" : "failed");
            #endif
        }


        ++entries_count;
        NRF_LOG_FLUSH();
    } while (fno.fname[0]);


    ff_result = f_closedir(&dir);

    #if MX_FLASH_DEBUG
        NRF_LOG_ERROR("close dir status: %u, %s", ff_result, (ff_result == FR_OK) ? "succeeded" : "failed");
    #endif

    #if 0// MX_FLASH_DEBUG
        ff_result = f_unlink("/");
        NRF_LOG_ERROR("Unlinking directory status: %u, %s", ff_result, (ff_result == FR_OK) ? "succeeded" : "failed");
        NRF_LOG_INFO("Entries count: %u", entries_count);
    #endif

    MXflash_FSdeinit();
    return ff_result;
}



static void fatfs_file_create(void)
{
    FRESULT ff_result;
    FIL file;
    

    NRF_LOG_INFO("Creating file: %s ...", (uint32_t)filename);
    NRF_LOG_FLUSH();

    ff_result = f_open(&file, filename, FA_CREATE_NEW | FA_WRITE);
    if(ff_result == FR_EXIST)
    {
        NRF_LOG_ERROR("File with name: %s already exists: %u", filename, ff_result);
        NRF_LOG_FLUSH();
        return;
    }
    else if (ff_result != FR_OK)
    {
        NRF_LOG_ERROR("Unable to open or create file: %u", ff_result);
        NRF_LOG_FLUSH();
        return;
    }

    ff_result = f_close(&file);
    if (ff_result != FR_OK)
    {
        NRF_LOG_ERROR("Unable to close file: %u", ff_result);
        NRF_LOG_FLUSH();
        return;
    }
    NRF_LOG_INFO("done creating file");
}



void MXflash_FSinit(void)
{
    if (fatfs_init())
    {
        // List the files in the filesystem
        fatfs_ls();
        fatfs_file_create();
        #if 0
            fatfs_read();
        #endif
    }
}


void MXflash_FSdeinit(void)
{
    fatfs_uninit();
    /* Fix for the MX waking-up on unint of the QSPI interface, despite setting the CSN pin as an output and high before calling the uninit. See:
     * https://devzone.nordicsemi.com/f/nordic-q-a/42609/disabling-qspi-draws-much-current-sdk15
     * https://devzone.nordicsemi.com/f/nordic-q-a/33563/qspi-drawing-more-power-when-disabled
     */
    flash_deepPowerDown();
}


