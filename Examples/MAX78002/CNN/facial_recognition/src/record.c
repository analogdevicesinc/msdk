/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/
#include <string.h>
#include <math.h>

#include "board.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc.h"
#include "utils.h"
#include "camera.h"
#include "faceID.h"
#include "MAXCAM_Debug.h"
#include "cnn_1.h"
#include "cnn_2.h"
#include "cnn_3.h"
#include "led.h"
#include "lp.h"
#include "uart.h"
#include "post_process.h"
#include "flc.h"
#include "facedetection.h"
#include "weights_3.h"
#include "baseaddr.h"
#include "record.h"
#include "embeddings.h"

#define S_MODULE_NAME "record"

/***** Definitions *****/
#define DB_ADDRESS                              \
    (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE) - \
        (5 *                                    \
         MXC_FLASH_PAGE_SIZE) // Max Flash use 72 * 1024 bytes, 5 pages of 16KB each are used for the database

#define STATUS_ADDRESS                          \
    (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE) - \
        (6 *                                    \
         MXC_FLASH_PAGE_SIZE) // Status is updated regularly, so we use a dedicated page for it
/*
    ^ Points to last page in flash, which is guaranteed to be unused by this small example.
    For larger applications it's recommended to reserve a dedicated flash region by creating
    a modified linkerfile.
*/
#define MAGIC 0xFEEDBEEF
#define TEST_VALUE 0xDEADBEEF

/*
Example Flash Database :
        MAGICKEY
        NNNNNNNN
        NNNNIIIL
        EEEEEEEE
        ........
        ........
        ........
        NNNNNNNN
        NNNNIIIL
        EEEEEEEE
        ........
        N : Name of the person (6 bytes)
        I : Unique ID of the person (12 bits)
        L : Embeddings count of the person (4 bits)
        E : Embeddings of the person (L * 64 bytes)

Example Status Field :
        TTTTTTTT
        CCCCCCCC
        T : Total number of people in the database (32 bits)
        C : Total embeddings count in the database (32 bits)
*/

struct person {
    char name[7];
    uint32_t id;
    uint32_t embeddings_count;
    uint32_t db_embeddings_count;
};

typedef struct person Person;

/***** Globals *****/
volatile uint32_t isr_cnt;
volatile uint32_t isr_flags;
volatile uint32_t db_flash_emb_count;
extern volatile char
    names[1024][7]; // 1024 names of 7 bytes each, as we support 1024 people in the database
extern volatile int32_t output_buffer[16];
extern unsigned int touch_x, touch_y;
extern volatile uint8_t face_detected;
extern volatile uint8_t capture_key;
extern volatile uint8_t record_mode;
volatile uint8_t face_ok = 0;
extern area_t area;
extern area_t area_1;
extern area_t area_2;
extern uint8_t box[4]; // x1, y1, x2, y2
static const uint32_t baseaddr[] = BASEADDR;
int key;
char alphabet[26][1] = { "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M",
                         "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z" };
static int font = (int)&Liberation_Sans16x16[0];

/***** Prototypes *****/
void get_status(Person *p);
int update_status(Person *p);
int update_info_field(Person *p);
void FLC0_IRQHandler(void);
int init_db(void);
int init_status(void);
int add_person(Person *p);
void flash_to_cnn(Person *p, uint32_t cnn_location);
void setup_irqs(void);
void read_db(Person *p);
bool check_db(void);
void show_keyboard(void);
void get_name(Person *p);
void show_face(void);

/***** Functions *****/

void get_name(Person *p)
{
    show_keyboard();
    int key = 0;
    int len = 0;
    text_t text_buffer;
    area_t area_buffer;

    strncpy(p->name, "\0\0\0\0\0\0", 7);

    while (1) {
        key = MXC_TS_GetKey();
        if (key == 0)
            continue;
        else if (key == 28) {
            p->name[len] = '\0'; // null terminator
            break;
        } else if (key == 27 && len > 0) {
            len--;
            p->name[len] = '\0';

            text_buffer.data = p->name;
            text_buffer.len = 6;
            area_buffer.y = 300;
            area_buffer.x = 120;
            area_buffer.h = 20;
            area_buffer.w = 100;
            MXC_TFT_ClearArea(&area_buffer, 4);
            MXC_TFT_PrintFont(120, 300, font, &text_buffer, NULL);
        } else if (key != 27 && len < 6) // 7th character is reserved for null terminator
        {
            printf("key: %d\n", key);
            p->name[len] = alphabet[key - 1][0];
            len++;
            text_buffer.data = p->name;
            text_buffer.len = 7;
            MXC_TFT_PrintFont(120, 300, font, &text_buffer, NULL);
        }
    }

    MXC_TFT_ClearScreen();
    MXC_TS_RemoveAllButton();
}

void show_keyboard()
{
    // Description : Shows the keyboard on the TFT screen
    //Clear the screen and the buttons
    MXC_TFT_ClearScreen();
    MXC_TS_RemoveAllButton();

    //Add the buttons
    area_t area_buffer;
    text_t text_buffer;

    for (int i = 0; i < 26; i++) {
        if (i / 4 == 0) {
            MXC_TS_AddButton(40, 180 - (i % 4) * 50, 70, 220 - (i % 4) * 50, i + 1);
            area_buffer.y = 20;
            area_buffer.x = 20 + (i % 4) * 50;
            area_buffer.h = 30;
            area_buffer.w = 40;
            text_buffer.data = alphabet[i];
            text_buffer.len = 1;
            MXC_TFT_FillRect(&area_buffer, 0xFD20);
            MXC_TFT_PrintFont(area_buffer.x + 15, area_buffer.y + 12, font, &text_buffer, NULL);
        } else if (i / 4 == 1) {
            MXC_TS_AddButton(80, 180 - (i % 4) * 50, 110, 220 - (i % 4) * 50, i + 1);
            area_buffer.y = 60;
            area_buffer.x = 20 + (i % 4) * 50;
            area_buffer.h = 30;
            area_buffer.w = 40;
            text_buffer.data = alphabet[i];
            text_buffer.len = 1;
            MXC_TFT_FillRect(&area_buffer, 0xED20);
            MXC_TFT_PrintFont(area_buffer.x + 15, area_buffer.y + 12, font, &text_buffer, NULL);
        } else if (i / 4 == 2) {
            MXC_TS_AddButton(120, 180 - (i % 4) * 50, 150, 220 - (i % 4) * 50, i + 1);
            area_buffer.y = 100;
            area_buffer.x = 20 + (i % 4) * 50;
            area_buffer.h = 30;
            area_buffer.w = 40;
            text_buffer.data = alphabet[i];
            text_buffer.len = 1;
            MXC_TFT_FillRect(&area_buffer, 0xDD20);
            MXC_TFT_PrintFont(area_buffer.x + 15, area_buffer.y + 12, font, &text_buffer, NULL);
        } else if (i / 4 == 3) {
            MXC_TS_AddButton(155, 180 - (i % 4) * 50, 185, 220 - (i % 4) * 50, i + 1);
            area_buffer.y = 140;
            area_buffer.x = 20 + (i % 4) * 50;
            area_buffer.h = 30;
            area_buffer.w = 40;
            text_buffer.data = alphabet[i];
            text_buffer.len = 1;
            MXC_TFT_FillRect(&area_buffer, 0xCD20);
            MXC_TFT_PrintFont(area_buffer.x + 15, area_buffer.y + 12, font, &text_buffer, NULL);
        } else if (i / 4 == 4) {
            MXC_TS_AddButton(190, 180 - (i % 4) * 50, 220, 220 - (i % 4) * 50, i + 1);
            area_buffer.y = 180;
            area_buffer.x = 20 + (i % 4) * 50;
            area_buffer.h = 30;
            area_buffer.w = 40;
            text_buffer.data = alphabet[i];
            text_buffer.len = 1;
            MXC_TFT_FillRect(&area_buffer, 0xBD20);
            MXC_TFT_PrintFont(area_buffer.x + 15, area_buffer.y + 12, font, &text_buffer, NULL);
        } else if (i / 4 == 5) {
            MXC_TS_AddButton(225, 180 - (i % 4) * 50, 255, 220 - (i % 4) * 50, i + 1);
            area_buffer.y = 220;
            area_buffer.x = 20 + (i % 4) * 50;
            area_buffer.h = 30;
            area_buffer.w = 40;
            text_buffer.data = alphabet[i];
            text_buffer.len = 1;
            MXC_TFT_FillRect(&area_buffer, 0xAD20);
            MXC_TFT_PrintFont(area_buffer.x + 15, area_buffer.y + 12, font, &text_buffer, NULL);
        } else if (i / 4 == 6) {
            MXC_TS_AddButton(260, 180 - (i % 4) * 50, 290, 220 - (i % 4) * 50, i + 1);
            area_buffer.y = 260;
            area_buffer.x = 20 + (i % 4) * 50;
            area_buffer.h = 30;
            area_buffer.w = 40;
            text_buffer.data = alphabet[i];
            text_buffer.len = 1;
            MXC_TFT_FillRect(&area_buffer, 0x9D20);
            MXC_TFT_PrintFont(area_buffer.x + 15, area_buffer.y + 12, font, &text_buffer, NULL);
        }
    }

    //Add Bckspc button
    MXC_TS_AddButton(265, 80, 295, 120, 27);
    area_buffer.y = 260;
    area_buffer.x = 120;
    area_buffer.h = 30;
    area_buffer.w = 40;
    text_buffer.data = "Bck";
    text_buffer.len = 3;
    MXC_TFT_FillRect(&area_buffer, 0x9D20);
    MXC_TFT_PrintFont(area_buffer.x, area_buffer.y + 12, font, &text_buffer, NULL);

    //Add OK button
    MXC_TS_AddButton(265, 30, 295, 70, 28);
    area_buffer.y = 260;
    area_buffer.x = 170;
    area_buffer.h = 30;
    area_buffer.w = 40;
    text_buffer.data = "OK";
    text_buffer.len = 2;
    MXC_TFT_FillRect(&area_buffer, 0x9D20);
    MXC_TFT_PrintFont(area_buffer.x + 5, area_buffer.y + 12, font, &text_buffer, NULL);
}

void read_db(Person *p)
{
    // Description : Reads the database from flash and populates the Person structure, works in loop

    uint32_t info_address;
    uint32_t first_info;
    uint32_t second_info;

    info_address = (DB_ADDRESS + 4) + ((p->id - 1) * 4 * 2) + (p->db_embeddings_count * 64);

    MXC_FLC_Read(info_address, &first_info, 4);
    MXC_FLC_Read(info_address + 4, &second_info, 4);

    p->name[0] = (first_info >> 24) & 0xFF;
    p->name[1] = (first_info >> 16) & 0xFF;
    p->name[2] = (first_info >> 8) & 0xFF;
    p->name[3] = (first_info)&0xFF;
    p->name[4] = (second_info >> 24) & 0xFF;
    p->name[5] = (second_info >> 16) & 0xFF;
    p->name[6] = '\0';
    p->id = (second_info >> 4) & 0xFFF;
    p->embeddings_count = (second_info)&0xF;
}

void init_cnn_from_flash()
{
    uint32_t location = 0;
    uint32_t counter = 0;

    if (!check_db()) {
        PR_DEBUG("No database found, skipping CNN database initialization\n");
        return;
    }
    PR_DEBUG("Initializing CNN database from flash\n");
    Person total;
    Person *total_ptr = &total;
    get_status(total_ptr);

    Person p;
    Person *pptr = &p;

    pptr->id = 1;
    pptr->embeddings_count = 0;
    pptr->db_embeddings_count = 0;

    for (uint32_t i = 1; i < total_ptr->id; i++) {
        read_db(pptr); //Get the name, id and embeddings count of the person
        PR_DEBUG("Reading person %s with id %d and embeddings count %d\n", pptr->name, pptr->id,
                 pptr->embeddings_count);

        counter = pptr->embeddings_count;
        pptr->embeddings_count = 0;
        for (uint32_t j = 0; j < counter; j++) {
            flash_to_cnn(pptr, location + DEFAULT_EMBS_NUM);

            location += 1;
            pptr->embeddings_count += 1;
        }
        pptr->db_embeddings_count = pptr->db_embeddings_count + counter;
        pptr->id += 1;
    }
}

void get_status(Person *p)
{
    uint32_t id;
    uint32_t count;

    MXC_FLC_Read(STATUS_ADDRESS, &id, 4);
    if (id == 0xFFFFFFFF) { // Flash is empty
        p->id = 1;
        p->embeddings_count = 0;
        p->db_embeddings_count = 0;
        db_flash_emb_count =
            p->db_embeddings_count; // Get the total embeddings count from the status field for post processing
        return;
    }

    p->id = id + 1;

    MXC_FLC_Read(STATUS_ADDRESS + 4, &count, 4);
    p->embeddings_count = 0; // Initialize to 0 for every new person
    p->db_embeddings_count = count;
    db_flash_emb_count =
        p->db_embeddings_count; // Get the total embeddings count from the status field for post processing
}

int update_status(Person *p)
{
    int err = 0;
    err = init_status();
    if (err) {
        printf("Failed to initialize status", err);
        return err;
    }
    err = MXC_FLC_Write32(STATUS_ADDRESS, p->id);
    if (err) {
        printf("Failed to write status (ID)", err);
        return err;
    }
    err = MXC_FLC_Write32(STATUS_ADDRESS + 4, p->db_embeddings_count);
    if (err) {
        printf("Failed to write status (Embeddings count)", err);
        return err;
    }
    return err;
}

int update_info_field(Person *p)
{
    int err = 0;

    uint32_t first_info = 0x00000000;
    uint32_t second_info = 0x00000000;
    uint32_t info_address;

    /*
    NNNNNNNN First info field
    NNNNIIIL Second info field
    */

    info_address = (DB_ADDRESS + 4) + ((p->id - 1) * 4 * 2) + (p->db_embeddings_count * 64);

    first_info = (p->name[0] << 24) | (p->name[1] << 16) | (p->name[2] << 8) | (p->name[3]);

    second_info = (p->name[4] << 24) | (p->name[5] << 16) | (p->id << 4) | (p->embeddings_count);

    err = MXC_FLC_Write32(info_address, first_info);
    if (err) {
        printf("Failed to write status to 0x%x, %d, (First info)", info_address, err);
        return err;
    }

    err = MXC_FLC_Write32(info_address + 4, second_info);
    if (err) {
        printf("Failed to write status to 0x%x, %d, (Second info)", info_address + 4, err);
        return err;
    }

    return err;
}

void FLC0_IRQHandler()
{
    uint32_t temp;
    isr_cnt++;
    temp = MXC_FLC0->intr;

    if (temp & MXC_F_FLC_INTR_DONE) {
        MXC_FLC0->intr &= ~MXC_F_FLC_INTR_DONE;
        PR_DEBUG(" -> Interrupt! (Flash operation done)\n\n");
    }

    if (temp & MXC_F_FLC_INTR_AF) {
        MXC_FLC0->intr &= ~MXC_F_FLC_INTR_AF;
        PR_DEBUG(" -> Interrupt! (Flash access failure)\n\n");
    }

    isr_flags = temp;
}

//============================================================================
int init_db()
{
    int err = 0;
    printf("Erasing page 64 of flash (addr 0x%x)...\n", DB_ADDRESS);
    for (int i = 0; i < 5; i++) {
        err = MXC_FLC_PageErase(DB_ADDRESS + (i * MXC_FLASH_PAGE_SIZE)); // Erase 5 pages
        if (err) {
            printf("Failed with error code %i\n", DB_ADDRESS, err);
            return err;
        }
    }

    PR_DEBUG("Magic Value not matched, Initializing flash\n");
    err = MXC_FLC_Write32(DB_ADDRESS, MAGIC);

    if (err) {
        printf("Failed to write magic value to 0x%x with error code %i!\n", DB_ADDRESS, err);
        return err;
    }

    return err;
}

int init_status()
{
    int err = 0;
    printf("Erasing page 63 of flash (addr 0x%x)...\n", STATUS_ADDRESS);
    err = MXC_FLC_PageErase(STATUS_ADDRESS);
    if (err) {
        printf("Failed with error code %i\n", STATUS_ADDRESS, err);
        return err;
    }

    return err;
}

//============================================================================
bool check_db()
{
    uint32_t magic_read = 0;
    //Check if database is empty
    MXC_FLC_Read(DB_ADDRESS, &magic_read, 4);
    PR_DEBUG("Magic Value at address 0x%x \tRead: 0x%x\n", DB_ADDRESS, magic_read);

    return (magic_read == MAGIC);
}
//============================================================================
void show_face()
{
    uint32_t imgLen;
    uint32_t w, h;
    uint8_t *raw;
    uint8_t img_buffer[HEIGHT_ID * WIDTH_ID * 2]; //112x112x2
    uint8_t *img_ptr = img_buffer;

    uint8_t x_loc;
    uint8_t y_loc;
    float y_prime;
    float x_prime;
    int adj_boxes[4];
    int diff = 0;

    uint8_t box_width = box[2] - box[0];
    uint8_t box_height = box[3] - box[1];

    MXC_TFT_ClearScreen();

    if (box_width > box_height) {
        diff = box_width - box_height;
        PR_DEBUG("width is bigger diff: %d", diff);
        PR_DEBUG("x1: %d, y1: %d, x2: %d, y2: %d", box[0], box[1], box[2], box[3]);
        if (diff % 2 == 0) {
            adj_boxes[1] = (int)box[1] - diff / 2;
            adj_boxes[3] = (int)box[3] + diff / 2;
        } else {
            adj_boxes[1] = (int)box[1] - diff / 2;
            adj_boxes[3] = (int)box[3] + diff / 2 + 1;
        }
        adj_boxes[0] = (int)box[0];
        adj_boxes[2] = (int)box[2];
        PR_DEBUG("ADJUSTED x1: %d, y1: %d, x2: %d, y2: %d", adj_boxes[0], adj_boxes[1],
                 adj_boxes[2], adj_boxes[3]);
    } else {
        diff = box_height - box_width;
        PR_DEBUG("height is bigger diff: %d", diff);
        PR_DEBUG("x1: %d, y1: %d, x2: %d, y2: %d", box[0], box[1], box[2], box[3]);
        if (diff % 2 == 0) {
            adj_boxes[0] = (int)box[0] - diff / 2;
            adj_boxes[2] = (int)box[2] + diff / 2;
        } else {
            adj_boxes[0] = (int)box[0] - diff / 2;
            adj_boxes[2] = (int)box[2] + diff / 2 + 1;
        }
        adj_boxes[1] = (int)box[1];
        adj_boxes[3] = (int)box[3];
        PR_DEBUG("ADJUSTED x1: %d, y1: %d, x2: %d, y2: %d", adj_boxes[0], adj_boxes[1],
                 adj_boxes[2], adj_boxes[3]);
    }

    int x1 = adj_boxes[0];
    int y1 = adj_boxes[1];
    box_height = adj_boxes[3] - adj_boxes[1];
    box_width = adj_boxes[2] - adj_boxes[0];

    camera_get_image(&raw, &imgLen, &w, &h);
    PR_DEBUG("w , h , %d, %d\n", w, h);
    uint8_t *data = raw;

    // Get the details of the image from the camera driver.

    for (int i = 0; i < HEIGHT_ID; i++) {
        y_prime = ((float)(i) / HEIGHT_ID) * box_height;
        y_loc = (uint8_t)(MIN(round(y_prime), box_height - 1));

        for (int j = 0; j < WIDTH_ID; j++) {
            x_prime = ((float)(j) / WIDTH_ID) * box_width;
            x_loc = (uint8_t)(MIN(round(x_prime), box_width - 1));
            if ((x1 + x_loc < 0) || (y1 + y_loc < 0) || (x1 + x_loc >= WIDTH_DET) ||
                (y1 + y_loc >= HEIGHT_DET)) {
                img_buffer[(j * BYTE_PER_PIXEL * HEIGHT_ID) + (i * BYTE_PER_PIXEL)] = 0;
                img_buffer[(j * BYTE_PER_PIXEL * HEIGHT_ID) + (i * BYTE_PER_PIXEL) + 1] = 0;
            } else {
                img_buffer[(j * BYTE_PER_PIXEL * HEIGHT_ID) + (i * BYTE_PER_PIXEL)] =
                    data[((x1 + x_loc) * BYTE_PER_PIXEL * HEIGHT_DET) +
                         ((y1 + y_loc) * BYTE_PER_PIXEL)];
                img_buffer[(j * BYTE_PER_PIXEL * HEIGHT_ID) + (i * BYTE_PER_PIXEL) + 1] =
                    data[((x1 + x_loc) * BYTE_PER_PIXEL * HEIGHT_DET) +
                         ((y1 + y_loc) * BYTE_PER_PIXEL) + 1];
            }
        }
    }

    MXC_TFT_SetRotation(ROTATE_270);
    __disable_irq(); // Disable IRQ to block communication with touch screen
    MXC_TFT_Stream(SHOW_START_X, SHOW_START_Y, HEIGHT_ID, WIDTH_ID);
    // Stream captured image to TFT display
    TFT_SPI_Transmit(img_ptr, HEIGHT_ID * WIDTH_ID * 2);
    __enable_irq(); // Enable IRQ to resume communication with touch screen
    MXC_TFT_SetRotation(ROTATE_180);
}
//============================================================================
int add_person(Person *p)
{
    int err = 0;
    int init_reshow = 0;
    int init_faceid = 0;
    int init_come_closer = 0;
    face_detected = 0;
    text_t text_buffer;

    if (p->embeddings_count == 0) {
        PR_DEBUG("Enter name: ");
#ifdef TS_ENABLE
        get_name(p); // Get the name from TS
#else
        scanf("%5s", p->name);
#endif
        PR_DEBUG("Name entered: %s\n", p->name);
    }

    MXC_TS_AddButton(260, 160, 320, 240, 2);

    MXC_TFT_FillRect(&area_2, 0xFD20);

    text_buffer.data = "Capture";
    text_buffer.len = 7;
    MXC_TFT_PrintFont(0, 270, font, &text_buffer, NULL);
    // Re-enable the ICC
    MXC_ICC_Enable(MXC_ICC0);
    while (!face_ok) // Get user's feedback for the captured image
    {
        init_faceid = 0;
        while (!face_detected || !capture_key) {
            if (!init_faceid) {
                MXC_TFT_ClearScreen();
                MXC_TS_RemoveAllButton();
                MXC_TS_AddButton(260, 160, 320, 240, 2);
                MXC_TFT_FillRect(&area_2, 0xFD20);
                text_buffer.data = "Capture";
                text_buffer.len = 7;
                MXC_TFT_PrintFont(0, 270, font, &text_buffer, NULL);
                init_faceid = 1;
            }

            face_detection();
            if (face_detected) {
                printf("Box width: %d\n", box[2] - box[0]);
                printf("Box height: %d\n", box[3] - box[1]);
                if ((box[2] - box[0]) < 70 || (box[3] - box[1]) < 110) {
                    face_detected = 0;

                    if (!init_come_closer) {
                        text_buffer.data = "Come Closer";
                        text_buffer.len = 11;
                        MXC_TFT_PrintFont(60, 300, font, &text_buffer, NULL);
                        init_come_closer = 1;
                    }

                    continue;
                } else {
                    MXC_TFT_ClearArea(&area, 4);
                    init_come_closer = 0;
                }
            }

#ifdef TS_ENABLE
            key = MXC_TS_GetKey();
            if (key == 2) {
                capture_key = 1;
                MXC_TFT_FillRect(&area_2, 0x9D20);
                text_buffer.data = "Capture";
                text_buffer.len = 7;
                MXC_TFT_PrintFont(0, 270, font, &text_buffer, NULL);
            }

#endif

            //face_detected = 0;
        }
        capture_key = 0;
        face_detected = 0;

        show_face();

        if (!init_reshow) {
            //Clear buttons
            MXC_TFT_FillRect(&area_1, 0xFD20);
            MXC_TFT_FillRect(&area_2, 0xFD20);
            MXC_TS_AddButton(260, 0, 320, 80, 1);
            MXC_TS_AddButton(260, 160, 320, 240, 2);

            text_buffer.data = "OK";
            text_buffer.len = 2;
            MXC_TFT_PrintFont(0, 270, font, &text_buffer, NULL);

            text_buffer.data = "Retry";
            text_buffer.len = 5;
            MXC_TFT_PrintFont(162, 270, font, &text_buffer, NULL);
            init_reshow = 1;
        }

        //Show captured face
        // Ask user if he/she is ok with the image
        // If not, repeat the process
        key = 0;
        while (key == 0) //Wait for user's feedback, 0 is the default value
        {
            key = MXC_TS_GetKey();
            if (key == 2) {
                face_ok = 1;
                init_reshow = 0;
                PR_DEBUG("Face is ok\n");
            } else if (key == 1) {
                face_ok = 0;
                init_reshow = 0;
                PR_DEBUG("Face is not ok\n");
            }
        }
    }
    face_ok = 0;
    capture_key = 0;
    MXC_ICC_Disable(MXC_ICC0); //Disable ICC for flash write
    face_id();

    PR_DEBUG("This is record\n");

    //Calculate the write address 4 bytes for magic key, 8 bytes for each person, 64 bytes for each embedding
    PR_DEBUG("p.id %d", p->id);
    PR_DEBUG("p.embeddings_count %d", p->embeddings_count);
    PR_DEBUG("Total embeddings_count %d", p->db_embeddings_count);
    uint32_t write_address = (DB_ADDRESS + 4) + ((p->id - 1) * 4 * 2) +
                             ((p->embeddings_count + p->db_embeddings_count) * 64);

    for (int i = 0; i < 16; i++) {
        //TODO: Update here for adjustable shift
        if (output_buffer[i] >= 128)
            output_buffer[i] = 256 - ((256 - output_buffer[i]) * CNN_3_OUTPUT_SHIFT);
        else
            output_buffer[i] = output_buffer[i] * CNN_3_OUTPUT_SHIFT;

        PR_DEBUG("Writing buffer value 0x%x to address 0x%x...\n", output_buffer[i],
                 write_address + ((i + 2) * 4)); // 2 for the name and length field

        err = MXC_FLC_Write32(write_address + ((i + 2) * 4), output_buffer[i]);

        if (err) {
            printf("Failed to write value to 0x%x with error code %i!\n",
                   write_address + ((i + 2) * 4), err);
            return err;
        }
    }

    flash_to_cnn(p, (p->embeddings_count + p->db_embeddings_count +
                     DEFAULT_EMBS_NUM)); // Load a single embedding into CNN_3
    p->embeddings_count += 1;

    record_mode = 0;
    PR_DEBUG("To continue to capture press P2.6, to return to main menu press P2.7\n");

    text_buffer.data = "Cnt?";
    text_buffer.len = 4;
    MXC_TFT_FillRect(&area_2, 0xFD20);
    MXC_TFT_PrintFont(0, 270, font, &text_buffer, NULL);

    MXC_TS_AddButton(260, 0, 320, 80, 1);

    MXC_TFT_FillRect(&area_1, 0xFD20);

    text_buffer.data = "Exit";
    text_buffer.len = 4;
    MXC_TFT_PrintFont(162, 270, font, &text_buffer, NULL);

    MXC_Delay(MSEC(1000));
    key = MXC_TS_GetKey(); //Dumb Buffer
    key = 0;
    capture_key = 0;
    while (!capture_key) {
#ifdef TS_ENABLE
        key = MXC_TS_GetKey();
        if (key == 1) {
            record_mode = 1;
        } else if (key == 2) {
            capture_key = 1;
        }
#endif
        if (record_mode) { //If record mode is off, return to main menu

            err = update_info_field(p); //Update the information field

            p->db_embeddings_count = p->db_embeddings_count + p->embeddings_count;

            MXC_TFT_ClearScreen();
            MXC_TS_RemoveAllButton();

            if (err) {
                printf("Failed to update info field with error code %i!\n", err);
                return err;
            }

            err = -1; // -1 is the exit code
            return err;
        }
    }

    capture_key = 0;
    MXC_TFT_ClearScreen();
    MXC_TS_RemoveAllButton();

    err = add_person(p);

    return err;
}

//============================================================================
void flash_to_cnn(Person *p, uint32_t cnn_location)
{
    volatile uint32_t *kernel_addr, *ptr;
    uint32_t readval = 0;

    uint32_t kernel_buffer[4];
    uint32_t write_buffer[3];
    uint32_t emb_buffer[16];
    uint32_t emb;
    uint32_t emb_addr;

    //Reload the latest emb
    int block_id = (cnn_location) / 9;
    int block_offset = (cnn_location) % 9;
    int write_offset = 8 - block_offset; // reverse order for each block;

    PR_DEBUG("Block ID: %d\tBlock Offset: %d\tWrite Offset: %d\n", block_id, block_offset,
             write_offset);

    emb_addr = (DB_ADDRESS + 4) + ((p->id) * 4 * 2) +
               ((p->embeddings_count + p->db_embeddings_count) *
                64); // 4 bytes for magic key, 8 bytes for each person, 64 bytes for each embedding

    //Read the emb from flash
    for (int i = 0; i < 16; i++) {
        MXC_FLC_Read(emb_addr + i * 4, &readval, 4);
        emb_buffer[i] = readval;
        PR_DEBUG("Read value 0x%x from address 0x%x\n", readval, emb_addr + i * 4);
    }

    //Write the kernel to CNN

    for (int base_id = 0; base_id < EMBEDDING_SIZE; base_id++) {
        emb = (emb_buffer[base_id / 4] << (8 * (3 - (base_id % 4)))) &
              0xFF000000; // 0xYYZZWWXX -> 0xXX000000, 0xWW000000, 0xZZ000000, 0xYY000000
        PR_DEBUG("Emb value: 0x%x\n", emb);
        kernel_addr = (volatile uint32_t *)(baseaddr[base_id] + block_id * 4);

        //Read the kernel from CNN first
        ptr = (volatile uint32_t *)(((uint32_t)kernel_addr & 0xffffe000) |
                                    (((uint32_t)kernel_addr & 0x1fff) << 2));

        for (int i = 0; i < 4; i++) {
            kernel_buffer[i] = *ptr;
            PR_DEBUG("Read value 0x%x from address 0x%x\n", kernel_buffer[i], (ptr));
            ptr++;
        }

        if (write_offset == 0) {
            write_buffer[0] =
                emb | (kernel_buffer[1] >> 8); // kernel buffer 0 is always in a shape of 0x000000XX
        } else if (write_offset == 1) {
            kernel_buffer[1] = ((kernel_buffer[1] & 0x00FFFFFF) | emb);
        } else if (write_offset == 2) {
            kernel_buffer[1] = ((kernel_buffer[1] & 0xFF00FFFF) | (emb >> 8));
        } else if (write_offset == 3) {
            kernel_buffer[1] = ((kernel_buffer[1] & 0xFFFF00FF) | (emb >> 16));
        } else if (write_offset == 4) {
            kernel_buffer[1] = ((kernel_buffer[1] & 0xFFFFFF00) | (emb >> 24));
        } else if (write_offset == 5) {
            kernel_buffer[2] = ((kernel_buffer[2] & 0x00FFFFFF) | emb);
        } else if (write_offset == 6) {
            kernel_buffer[2] = ((kernel_buffer[2] & 0xFF00FFFF) | (emb >> 8));
        } else if (write_offset == 7) {
            kernel_buffer[2] = ((kernel_buffer[2] & 0xFFFF00FF) | (emb >> 16));
        } else if (write_offset == 8) {
            kernel_buffer[2] = ((kernel_buffer[2] & 0xFFFFFF00) | (emb >> 24));
        }

        *((volatile uint8_t *)((uint32_t)kernel_addr | 1)) = 0x01; // Set address

        if (write_offset != 0) {
            write_buffer[0] =
                (kernel_buffer[0] << 24) |
                (kernel_buffer[1] >> 8); // kernel buffer 0 is always in a shape of 0x000000XX
            PR_DEBUG("Write buffer 0: 0x%x\n", write_buffer[0]);
        }

        write_buffer[1] = (kernel_buffer[1] << 24) | (kernel_buffer[2] >> 8);
        PR_DEBUG("Write buffer 1: 0x%x\n", write_buffer[1]);
        write_buffer[2] = (kernel_buffer[2] << 24) | (kernel_buffer[3] >> 8);
        PR_DEBUG("Write buffer 2: 0x%x\n", write_buffer[2]);

        // 4 is always empty, so we don't need to write it
        for (int i = 0; i < 3; i++) {
            PR_DEBUG("Writing value 0x%x to address 0x%x\n", write_buffer[i], kernel_addr);
            *kernel_addr++ = write_buffer[i];
        }
    }
    strncpy((char *)names[cnn_location], p->name, 7);
}

void setup_irqs()
{
    /*
    All functions modifying flash contents are set to execute out of RAM
    with the (section(".flashprog")) attribute.  Therefore,

    If:
    - An FLC function is in the middle of execution (from RAM)
    ... and...
    - An interrupt triggers an ISR which executes from Flash

    ... Then a hard fault will be triggered.

    FLC functions should be:
    1) Executed from a critical code block (interrupts disabled)
    or
    2) ISRs should be set to execute out of RAM with NVIC_SetRAM()

    This example demonstrates method #1.  Any code modifying
    flash is executed from a critical block, and the FLC
    interrupts will trigger afterwards.
    */

    // NVIC_SetRAM(); // Execute ISRs out of SRAM (for use with #2 above)
    MXC_NVIC_SetVector(FLC0_IRQn, FLC0_IRQHandler); // Assign ISR
    NVIC_EnableIRQ(FLC0_IRQn); // Enable interrupt

    __enable_irq();

    // Clear and enable flash programming interrupts
    MXC_FLC_EnableInt(MXC_F_FLC_INTR_DONEIE | MXC_F_FLC_INTR_AFIE);
    isr_flags = 0;
    isr_cnt = 0;
}

int record()
{
    Person p;
    Person *pptr = &p;
    text_t text_buffer;

    setup_irqs(); // See notes in function definition

    /*
    Disable the instruction cache controller (ICC).

    Any code that modifies flash contents should disable the ICC,
    since modifying flash contents may invalidate cached instructions.
    */
    MXC_ICC_Disable(MXC_ICC0);

    int err = 0;

    if (check_db()) {
        PR_DEBUG(
            "Magic Matched, Database found\n"); //if magic value matches, get the latest ID from flash

    }

    else //if magic value does not match, initialize flash
    {
        err = init_db();
        if (err) {
            printf("Failed to initialize database", err);
            return err;
        }
        err = init_status();
        if (err) {
            printf("Failed to initialize status", err);
            return err;
        }
    }

    get_status(pptr);
    PR_DEBUG("Latest ID: %d\n", pptr->id);
    PR_DEBUG("Total embeddings: %d\n", pptr->db_embeddings_count);
    if (pptr->db_embeddings_count == 1024) {
        printf("Database full, can't add more persons\n");
        return -1;
    }

    err = add_person(pptr);
    if (err == -1) { // error code -1 means return to main menu
        err = update_status(pptr);
        if (err) {
            printf("Failed to update status", err);
            return err;
        }
        get_status(pptr);
        printf("Exiting to main menu", err);
        // Re-enable the ICC
        MXC_ICC_Enable(MXC_ICC0);
        printf("Successfully verified test pattern!\n\n");
        return err;
    } else if (err == -7) //Write exception handler
    {
        MXC_TFT_ClearScreen();
        text_buffer.data = "DB Corrupted, Reinitializing";
        text_buffer.len = 29;
        MXC_TFT_PrintFont(0, 270, font, &text_buffer, NULL);
        PR_DEBUG("Database Corrupted\n");
        err = init_db();
        if (err) {
            printf("Failed to initialize database", err);
            return err;
        }
        err = init_status();
        if (err) {
            printf("Failed to initialize status", err);
            return err;
        }

        // Reload default weights
        cnn_3_load_weights();
        cnn_3_configure();

        MXC_TFT_ClearScreen();

    }

    else if (err != 0) {
        printf("Failed to add person", err);
        return err;
    }

    return err;
}
