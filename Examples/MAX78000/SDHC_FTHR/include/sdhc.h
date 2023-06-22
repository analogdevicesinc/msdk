#ifndef SDHC_HEADER
#define SDHC_HEADER

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "gpio.h"
#include "uart.h"

#include "ff.h"
#include "cli.h"

/***** Definitions *****/

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define MAXLEN 256
/***** Globals *****/
FATFS *fs; //FFat Filesystem Object
FATFS fs_obj;
FIL file; //FFat File Object
FRESULT err; //FFat Result (Struct)
FILINFO fno; //FFat File Information Object
DIR dir; //FFat Directory Object

/***** FUNCTION PROTOTYPES *****/

void generateMessage(unsigned length);

int mount();

int umount();

int formatSDHC();

int getSize();

int ls();

int createFile();

int appendFile();

int mkdir();

int cd();

int delete();

int example();

void waitCardInserted();


#endif //SDHC_HEADER