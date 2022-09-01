/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
#include "embedding_process.h"
#include "MAXCAM_Debug.h"
#include "board.h"
#include "embeddings.h"
#include "mxc_device.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define S_MODULE_NAME "embedding"

/*******************************      DEFINES      ***************************/

static const uint8_t embeddings[] = EMBEDDINGS;

typedef struct __attribute__((packed)) {
    uint8_t numberOfSubjects;
    uint16_t lengthOfEmbeddings;
    uint16_t numberOfEmbeddings;
    uint16_t imageWidth;
    uint16_t imageHeight;
    uint16_t lengthOfSubjectNames;

} tsFaceIDFile;

tsFaceIDFile* pDatabaseInfo = NULL;

typedef struct __attribute__((packed)) sDistance {
    uint8_t subID;
    int32_t distance;
} tsDistance;

tsDistance* pDistance = NULL;

tsMeanDistance* pMeanDistance = NULL;
tsMinDistance* pMinDistance = NULL;

int8_t* pClosestSubId = NULL;
uint32_t closestSubIdBufIdx = 0;

uint8_t* pMinDistanceCounter = NULL;

/******************************** Static Functions ***************************/

/******************************** Public Functions ***************************/
int init_database(void)
{
    pDatabaseInfo = (tsFaceIDFile*)embeddings;

    pDistance = (tsDistance*)malloc(sizeof(tsDistance) * pDatabaseInfo->numberOfEmbeddings);

    if (pDistance == NULL) {
        return -1;
    }

    pMeanDistance
        = (tsMeanDistance*)malloc(sizeof(tsMeanDistance) * pDatabaseInfo->numberOfSubjects);

    if (pMeanDistance == NULL) {
        return -1;
    }

    pMinDistance = (tsMinDistance*)malloc(sizeof(tsMinDistance) * 3);

    if (pMinDistance == NULL) {
        return -1;
    }

    pClosestSubId = (int8_t*)malloc(closest_sub_buffer_size);

    if (pClosestSubId == NULL) {
        return -1;
    }

    for (int i = 0; i < closest_sub_buffer_size; ++i) { pClosestSubId[i] = -1; }

    pMinDistanceCounter = (uint8_t*)malloc(pDatabaseInfo->numberOfSubjects);

    if (pMinDistanceCounter == NULL) {
        return -1;
    }

    for (int i = 0; i < pDatabaseInfo->numberOfSubjects; ++i) { pMinDistanceCounter[i] = 0; }

    return 0;
}

char* get_subject(int ID)
{
    char* point = (char*)(pDatabaseInfo + 1);

    if (ID == 0) {
        return point;
    }

    int counter = 0;

    for (int i = 0; i < pDatabaseInfo->lengthOfSubjectNames; i++) {
        point++;

        if (*point == '\0') {
            counter++;

            if (counter == ID) {
                point++;
                return point;
            }
        }
    }

    return 0;
}

void get_min_dist_counter(uint8_t** counter, uint8_t* counter_len)
{
    *counter = pMinDistanceCounter;
    *counter_len = pDatabaseInfo->numberOfSubjects;
}

tsMinDistance* get_min_distance()
{
    return pMinDistance;
}

int calculate_minDistance(const uint8_t* embedding)
{
    int8_t* theEmbedding = (int8_t*)embedding;
    int8_t* theEmbeddingOrigin = theEmbedding;

    tsDistance* dist = pDistance;
    tsMeanDistance* meanDist = pMeanDistance;

    int8_t* pData = (int8_t*)((uint32_t)(pDatabaseInfo + 1) + pDatabaseInfo->lengthOfSubjectNames);

    // Calculate min distance for each embedding
    for (int i = 0; i < pDatabaseInfo->numberOfEmbeddings; i++) {
        int total = 0;
        dist->subID = (uint8_t)(*(pData++));
        theEmbedding = theEmbeddingOrigin;

        for (int j = 0; j < pDatabaseInfo->lengthOfEmbeddings; j++) {
            total += abs((*(theEmbedding++)) - (*(pData++)));
        }

        dist->distance = total;
        dist++;
    }

    dist = pDistance;

    for (int i = 0; i < pDatabaseInfo->numberOfSubjects; i++) {
        meanDist[i].subID = i;
        meanDist[i].number = 0;
        meanDist[i].distance = 0;
    }

    for (int i = 0; i < pDatabaseInfo->numberOfEmbeddings; i++) {
        meanDist = pMeanDistance + dist->subID;
        meanDist->distance += dist->distance;
        meanDist->number++;
        dist++;
    }

    meanDist = pMeanDistance;

    for (int i = 0; i < pDatabaseInfo->numberOfSubjects; i++) {
        meanDist->distance = meanDist->distance / meanDist->number;
        meanDist++;
    }

    meanDist = pMeanDistance;

#define MAX_DISTANCE 1000000

    for (int i = 0; i < 3; i++) {
        pMinDistance[i].subID = 0xFF;
        pMinDistance[i].distance = MAX_DISTANCE;
    }

    for (int i = 0; i < pDatabaseInfo->numberOfSubjects; i++) {
        if (meanDist[i].distance
            < pMinDistance[0].distance) { /* Check if current element is less than firstMin, then
                                             update first, second and third */
            pMinDistance[2].distance = pMinDistance[1].distance;
            pMinDistance[2].subID = pMinDistance[1].subID;
            pMinDistance[1].distance = pMinDistance[0].distance;
            pMinDistance[1].subID = pMinDistance[0].subID;
            pMinDistance[0].distance = meanDist[i].distance;
            pMinDistance[0].subID = meanDist[i].subID;
        } else if (meanDist[i].distance
            < pMinDistance[1].distance) { /* Check if current element is less than secmin then
                                             update second and third */
            pMinDistance[2].distance = pMinDistance[1].distance;
            pMinDistance[2].subID = pMinDistance[1].subID;
            pMinDistance[1].distance = meanDist[i].distance;
            pMinDistance[1].subID = meanDist[i].subID;
        } else if (meanDist[i].distance
            < pMinDistance[2]
                  .distance) { /* Check if current element is less than then update third */
            pMinDistance[2].distance = meanDist[i].distance;
            pMinDistance[2].subID = meanDist[i].subID;
        }
    }

    uint32_t bufferIdx = closestSubIdBufIdx % (closest_sub_buffer_size);

    if (pClosestSubId[bufferIdx] >= 0) {
        --pMinDistanceCounter[pClosestSubId[bufferIdx]];
    }

    if (pMinDistance[0].distance < thresh_for_unknown_subject) {
        pClosestSubId[bufferIdx] = pMinDistance[0].subID;
        ++pMinDistanceCounter[pClosestSubId[bufferIdx]];
    } else {
        pClosestSubId[bufferIdx] = -1;
    }

    ++closestSubIdBufIdx;

    PR_INFO("Results:\n");
    PR_INFO("1. : %d, distance: %d\n", pMinDistance[0].subID, pMinDistance[0].distance);
    PR_INFO("2. : %d, distance: %d\n", pMinDistance[1].subID, pMinDistance[1].distance);
    PR_INFO("3. : %d, distance: %d\n", pMinDistance[2].subID, pMinDistance[2].distance);
    PR_INFO("\n");

    PR_INFO("\t");

    for (int i = 0; i < closest_sub_buffer_size; ++i) { PR_INFO("%d, ", pClosestSubId[i]); }

    PR_INFO("\nIdx: %d, Buffer Idx: %d\n", closestSubIdBufIdx, bufferIdx);

    for (int i = 0; i < pDatabaseInfo->numberOfSubjects; ++i) {
        PR_INFO("\tId %d: %d\n", i, pMinDistanceCounter[i]);
    }

    PR_INFO("\n\n");

    return (closestSubIdBufIdx % 3);
}
