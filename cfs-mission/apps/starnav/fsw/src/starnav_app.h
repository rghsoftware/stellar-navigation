/*
 * MIT License
 *
 * Copyright (c) 2025 Robert Hamilton
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file starnav_app.h
 */

#ifndef STARNAV_APP_H
#define STARNAV_APP_H

/*
** Include Files
*/
#include "starnav_msg.h"
#include "starnav_msgid.h"

#include "cfe.h"
#include "common_types.h"

#include <stdint.h>

/*
** Type Definitions
*/
/* Performance IDs */
#define STARNAV_PERF_ID (91)

/* Event IDs */
#define STARNAV_INIT_INF_EID (1)
#define STARNAV_PIPE_ERR_EID (2)
#define STARNAV_SUB_ERR_EID  (3)
#define STARNAV_DEV_ERR_EID  (4)
#define STARNAV_DEV_INF_EID  (5)

/* Pipe depths */
#define STARNAV_PIPE_DEPTH (32)

/* Return codes */
#define STARNAV_NO_DATA (1)

/**
 * @brief StarNav Application Global Data Structure
 *
 * Contains all runtime state and working data for the StarNav application,
 * including software bus interfaces, telemetry buffers, and device handles.
 *
 * This structure is instantiated once as a global variable and maintains
 * all application state throughout the application lifecycle.
 */
typedef struct
{
    CFE_SB_PipeId_t CommandPipe; /**< Software bus pipe for receiving commands */
    uint32          RunStatus;   /**< Application run status (CFE_ES_RunStatus_APP_RUN, etc.) */

    /* Telemetry buffers */
    STARNAV_HkTlm_t       HkTlm;       /**< Housekeeping telemetry message buffer */
    STARNAV_AttitudeTlm_t AttitudeTlm; /**< Attitude data telemetry message buffer */
    STARNAV_StatusTlm_t   StatusTlm;   /**< Status telemetry message buffer */

    /* Device handle */
    osal_id_t DeviceFd; /**< File descriptor for UART device connection to STM32 star tracker */
} STARNAV_AppData_t;

/* Global data declaration */
extern STARNAV_AppData_t STARNAV_AppData;

/*
** Exported Functions
*/
void         STARNAV_Main(void);
CFE_Status_t STARNAV_Init(void);
void         STARNAV_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr);
void         STARNAV_ProcessDeviceData(void);

/* Device functions */
CFE_Status_t STARNAV_InitDevice(void);
CFE_Status_t STARNAV_ReadDevice(void);
void         STARNAV_ProcessPacket(uint8_t *packet, uint16_t length);

#endif /* STARNAV_APP_H */
