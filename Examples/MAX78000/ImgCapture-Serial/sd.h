#ifndef SD_H
#define SD_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "ff.h"
// http://elm-chan.org/fsw/ff/00index_e.html

#define MAXLEN 256

extern FATFS* sd_fs; //FFat Filesystem Object
extern FIL sd_file;    //FFat File Object
extern FRESULT sd_err; //FFat Result (Struct)
extern FILINFO sd_fno; //FFat File Information Object
extern DIR sd_dir;     //FFat Directory Object
extern TCHAR sd_message[MAXLEN], sd_directory[MAXLEN], sd_cwd[MAXLEN], sd_filename[MAXLEN], sd_volume_label[24],
    sd_volume;
extern DWORD sd_clusters_free, sd_sectors_free, sd_sectors_total, sd_volume_sn;
extern UINT sd_bytes_written, sd_bytes_read, sd_mounted;
extern BYTE sd_work[4096];
extern TCHAR* FR_ERRORS[20];

FRESULT sd_mount();
FRESULT sd_get_size();
FRESULT sd_get_cwd();
UINT out_stream (const BYTE *p, UINT btf);

#endif