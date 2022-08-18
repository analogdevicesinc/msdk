#include "sd.h"

FATFS fatfs; // Private fatFS object.

FATFS* sd_fs;   //FFat Filesystem Object
FIL sd_file;    //FFat File Object
FRESULT sd_err; //FFat Result (Struct)
FILINFO sd_fno; //FFat File Information Object
DIR sd_dir;     //FFat Directory Object
TCHAR sd_message[MAXLEN], sd_directory[MAXLEN], sd_cwd[MAXLEN], sd_filename[MAXLEN],
    sd_volume_label[24], sd_volume = '0';
DWORD sd_clusters_free = 0, sd_sectors_free = 0, sd_sectors_total = 0, sd_volume_sn = 0;
UINT sd_bytes_written = 0, sd_bytes_read = 0, sd_mounted = 0;
BYTE sd_work[4096];
TCHAR* FR_ERRORS[20] = {"FR_OK",
                        "FR_DISK_ERR",
                        "FR_INT_ERR",
                        "FR_NOT_READY",
                        "FR_NO_FILE",
                        "FR_NO_PATH",
                        "FR_INVLAID_NAME",
                        "FR_DENIED",
                        "FR_EXIST",
                        "FR_INVALID_OBJECT",
                        "FR_WRITE_PROTECTED",
                        "FR_INVALID_DRIVE",
                        "FR_NOT_ENABLED",
                        "FR_NO_FILESYSTEM",
                        "FR_MKFS_ABORTED",
                        "FR_TIMEOUT",
                        "FR_LOCKED",
                        "FR_NOT_ENOUGH_CORE",
                        "FR_TOO_MANY_OPEN_FILES",
                        "FR_INVALID_PARAMETER"};

FRESULT sd_mount()
{
    // Set the sd_fs pointer to the private fatfs object.  This "initializes" fatfs.
    sd_fs = &fatfs;

    printf("Mounting SD card...\n");

    // http://elm-chan.org/fsw/ff/doc/mount.html
    if ((sd_err = f_mount(sd_fs, "", 1)) != FR_OK) {
        printf("Error mounting SD card: %s\n", FR_ERRORS[sd_err]);
        f_mount(NULL, "", 0);
        return sd_err;
    } else {
        printf("SD card mounted.\n");
        sd_mounted = 1;
    }

    // If the label of the default drive is empty (""), then
    // reformat the card to exFAT and label the default drive.
    f_getlabel(&sd_volume, sd_volume_label, &sd_volume_sn);
    if (strcmp("", sd_volume_label) == 0) {
        printf("Empty volume label detected.\n");
        printf("Reformatting...\n");

        // http://elm-chan.org/fsw/ff/doc/mkfs.html
        // Attempt to mount with exFAT
        if ((sd_err = f_mkfs("", FM_FAT32, 0, sd_work, sizeof(sd_work))) != FR_OK) {
            printf("Error formatting card: %s\n", FR_ERRORS[sd_err]);
            return sd_err;
        }
        printf("Done!\n");

        f_setlabel("MAXIM-SD"); // http://elm-chan.org/fsw/ff/doc/setlabel.html
        printf("Set volume label.\n");
    }

    if ((sd_err = f_getlabel("", sd_volume_label, &sd_volume_sn)) != FR_OK) {
        printf("Error retrieving volume label: %s\n", FR_ERRORS[sd_err]);
        return sd_err;
    } else {
        printf("Volume label: %s\n", sd_volume_label);
    }

    return FR_OK;
}

FRESULT sd_unmount()
{
    sd_err = f_unmount("");
    if (sd_err != FR_OK) {
        printf("Error unmounting SD card: %s\n", FR_ERRORS[sd_err]);
    }
    printf("SD card unmounted.\n");
    return sd_err;
}

FRESULT sd_get_size()
{
    if ((sd_err = f_getfree(&sd_volume, &sd_clusters_free, &sd_fs)) != FR_OK) {
        printf("Error finding free size of card: %s\n", FR_ERRORS[sd_err]);
    }

    sd_sectors_total = (sd_fs->n_fatent - 2) * sd_fs->csize; // Calculate total size (in bytes)
    sd_sectors_free  = sd_clusters_free * sd_fs->csize;      // Calculate free space (in byte)

    return sd_err;
}

FRESULT sd_get_cwd()
{
    sd_err = f_getcwd(sd_cwd, MAXLEN);
    if (sd_err != FR_OK) {
        printf("Error getting cwd: %s\n", FR_ERRORS[sd_err]);
    }

    return sd_err;
}

FRESULT sd_cd(const char* dir)
{
    sd_err = f_chdir((const TCHAR*)dir);
    if (sd_err != FR_OK) {
        printf("Error changing directory: %s\n", FR_ERRORS[sd_err]);
    }

    // Refresh sd_cwd variable.
    sd_get_cwd();

    return sd_err;
}

FRESULT sd_ls()
{
    // List the contents of the current directory
    sd_err = f_opendir(&sd_dir, sd_cwd);
    if (sd_err != FR_OK) {
        printf("Error opening directory: %s\n", FR_ERRORS[sd_err]);
    } else {
        printf(".\n");
        for (;;) {
            sd_err = f_readdir(&sd_dir, &sd_fno);
            if (sd_err != FR_OK || sd_fno.fname[0] == 0)
                break;
            if (sd_fno.fattrib & AM_DIR) {
                printf("%s/\n", sd_fno.fname);
            } else {
                printf("%s\n", sd_fno.fname);
            }
        }
        f_closedir(&sd_dir);
    }
    return sd_err;
}

FRESULT sd_mkdir(const char* dir)
{
    // Make a directory
    sd_err = f_mkdir((const TCHAR*)dir);
    if (sd_err != FR_OK) {
        printf("Error creating directory: %i\n", FR_ERRORS[sd_err]);
    }
    return sd_err;
}

FRESULT sd_rm(const char* item)
{
    sd_err = f_unlink((const TCHAR*)item);
    if (sd_err != FR_OK) {
        printf("Error while deleting: %s\n", FR_ERRORS[sd_err]);
    }
    return sd_err;
}

FRESULT sd_touch(const char* filepath)
{
    sd_err = f_open(&sd_file, (const TCHAR*)filepath, FA_CREATE_NEW);
    if (sd_err != FR_OK) {
        printf("Error creating file: %s\n", FR_ERRORS[sd_err]);
    }
    f_close(&sd_file);
    return sd_err;
}

FRESULT sd_write_string(const char* filepath, const char* string)
{
    int len    = strlen(string);
    UINT wrote = 0;
    sd_err     = f_open(&sd_file, (const TCHAR*)filepath, FA_WRITE);
    if (sd_err != FR_OK) {
        printf("Error opening file: %s\n", FR_ERRORS[sd_err]);
    } else {
        sd_err = f_write(&sd_file, string, len, &wrote);
        if (sd_err != FR_OK || wrote != len) {
            printf("Failed to write to file: %s\n", FR_ERRORS[sd_err]);
        }
    }
    f_close(&sd_file);
    return sd_err;
}

FRESULT sd_write(const char* filepath, const uint8_t* data, int len)
{
    UINT wrote = 0;
    sd_err = f_open(&sd_file, (const TCHAR*)filepath, FA_WRITE | FA_OPEN_APPEND | FA_CREATE_NEW);
    if (sd_err != FR_OK) {
        printf("Error opening file: %s\n", FR_ERRORS[sd_err]);
    } else {
        sd_err = f_write(&sd_file, data, len, &wrote);
        if (sd_err != FR_OK || wrote != len) {
            printf("Failed to write to file: %s\n", FR_ERRORS[sd_err]);
        }
    }
    f_close(&sd_file);
    return sd_err;
}

FRESULT sd_cat(const char* filepath)
{
    sd_err = f_open(&sd_file, (const TCHAR*)filepath, FA_READ);
    if (sd_err != FR_OK) {
        printf("Error opening file: %s\n", FR_ERRORS[sd_err]);
    } else {
        while (sd_err == FR_OK && !f_eof(&sd_file)) {
            sd_err = f_forward(&sd_file, out_stream, 1, NULL); // Stream to UART 1 byte at a time
        }
        // printf("\n"); // Cap the message with a newline for the host console
    }
    f_close(&sd_file);
    return sd_err;
}