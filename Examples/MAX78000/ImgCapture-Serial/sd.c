#include "sd.h"

FATFS fatfs; // Private fatFS object.

FATFS* sd_fs; //FFat Filesystem Object
FIL sd_file;    //FFat File Object
FRESULT sd_err; //FFat Result (Struct)
FILINFO sd_fno; //FFat File Information Object
DIR sd_dir;     //FFat Directory Object
TCHAR sd_message[MAXLEN], sd_directory[MAXLEN], sd_cwd[MAXLEN], sd_filename[MAXLEN], sd_volume_label[24],
    sd_volume = '0';
DWORD sd_clusters_free = 0, sd_sectors_free = 0, sd_sectors_total = 0, sd_volume_sn = 0;
UINT sd_bytes_written = 0, sd_bytes_read = 0, sd_mounted = 0;
BYTE sd_work[4096];
TCHAR* FR_ERRORS[20] = {
    "FR_OK",
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
    "FR_INVALID_PARAMETER"
};

FRESULT sd_mount() {
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
        if ((sd_err =f_mkfs("", FM_FAT32, 0, sd_work, sizeof(sd_work))) != FR_OK) {
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

FRESULT sd_get_size() {
    if ((sd_err = f_getfree(&sd_volume, &sd_clusters_free, &sd_fs)) != FR_OK) {
        printf("Error finding free size of card: %s\n", FR_ERRORS[sd_err]);
    }

    sd_sectors_total = (sd_fs->n_fatent - 2) * sd_fs->csize;
    sd_sectors_free  = sd_clusters_free * sd_fs->csize;

    return sd_err;
}

FRESULT sd_get_cwd() {
    if ((sd_err = f_getcwd(sd_cwd, MAXLEN)) != FR_OK) {
        printf("Error retrieving cwd: %s\n", FR_ERRORS[sd_err]);
    }

    return sd_err;
}

FRESULT sd_cd(const char* dir) {
    if ((sd_err = f_chdir((const TCHAR*)dir)) != FR_OK) {
        printf("Error changing directory: %s\n", FR_ERRORS[sd_err]);
    }

    return sd_err;
}