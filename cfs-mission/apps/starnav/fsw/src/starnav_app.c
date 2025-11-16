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
 * @file starnav_app.c
 * @brief StarNav Application Main Implementation
 *
 * This file contains the main implementation for the StarNav application,
 * which interfaces with a star tracker device via UART to provide attitude
 * determination data to the Core Flight System (cFS).
 *
 * The application follows the standard cFS application structure:
 * - Initialization
 * - Main processing loop
 * - Command and telemetry handling
 * - Device data processing
 */

/*
** Include Files:
*/
#include "starnav_app.h"

#include "cfe.h"

#include <string.h>

/*
** Global Data
*/
/** @brief Global application data structure instance */
STARNAV_AppData_t STARNAV_AppData;

/*
** Functions
*/

/**
 * @brief StarNav Application Main Entry Point
 *
 * This is the main entry point for the StarNav application. It performs
 * initialization, then enters the main processing loop where it:
 * - Waits for commands on the Software Bus
 * - Processes received commands
 * - Reads device data periodically during timeouts
 * - Handles errors and exits appropriately
 *
 * The main loop continues until the application receives a shutdown command
 * or encounters a critical error.
 *
 * @par Performance ID
 * STARNAV_PERF_ID is used for performance monitoring
 *
 * @return None (function does not return; exits via CFE_ES_ExitApp)
 *
 * @note This function should only be called once by the cFS Executive Services
 * @see STARNAV_Init, STARNAV_ProcessCommandPacket, STARNAV_ProcessDeviceData
 */
void STARNAV_Main(void)
{
    CFE_Status_t     status   = CFE_STATUS_INCORRECT_STATE;
    CFE_SB_Buffer_t *SBBufPtr = NULL;

    CFE_ES_PerfLogEntry(STARNAV_PERF_ID);

    /* Initialize application */
    status = STARNAV_Init();
    if (status != CFE_SUCCESS)
    {
        STARNAV_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /* Main loop */
    while (CFE_ES_RunLoop(&STARNAV_AppData.RunStatus))
    {
        CFE_ES_PerfLogExit(STARNAV_PERF_ID);

        /* Wait for command (with timeout) */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, STARNAV_AppData.CommandPipe, 100);

        CFE_ES_PerfLogEntry(STARNAV_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            STARNAV_ProcessCommandPacket(SBBufPtr);
        }
        else if (status == CFE_SB_TIME_OUT)
        {
            /* Normal timeout - read device data */
            STARNAV_ProcessDeviceData();
        }
        else
        {
            CFE_EVS_SendEvent(STARNAV_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "SB read error: 0x%08X",
                              (unsigned int)status);
            STARNAV_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    CFE_ES_ExitApp(STARNAV_AppData.RunStatus);
}

/**
 * @brief Initialize the StarNav Application
 *
 * Performs complete initialization of the StarNav application including:
 * - Clearing application data structure
 * - Registering with Event Services
 * - Creating Software Bus command pipe
 * - Subscribing to housekeeping and command messages
 * - Initializing telemetry message headers
 * - Initializing the UART device connection
 *
 * This function is called once during application startup by STARNAV_Main.
 *
 * @return CFE_Status_t
 * @retval CFE_SUCCESS Initialization completed successfully
 * @retval Other Error codes from CFE_EVS_Register, CFE_SB_CreatePipe,
 *               CFE_SB_Subscribe, or STARNAV_InitDevice
 *
 * @note On failure, the calling function should set RunStatus to APP_ERROR
 * @see STARNAV_Main, STARNAV_InitDevice
 */
CFE_Status_t STARNAV_Init(void)
{
    CFE_Status_t status = CFE_STATUS_INCORRECT_STATE;

    memset(&STARNAV_AppData, 0, sizeof(STARNAV_AppData));
    STARNAV_AppData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /* Register events */
    status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
        return status;

    /* Create Software Bus pipe */
    status = CFE_SB_CreatePipe(&STARNAV_AppData.CommandPipe, STARNAV_PIPE_DEPTH, "STARNAV_CMD_PIPE");
    if (status != CFE_SUCCESS)
        return status;

    /* Subscribe to housekeeping requests */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(STARNAV_SEND_HK_MID), STARNAV_AppData.CommandPipe);
    if (status != CFE_SUCCESS)
        return status;

    /* Subscribe to ground commands */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(STARNAV_CMD_MID), STARNAV_AppData.CommandPipe);
    if (status != CFE_SUCCESS)
        return status;

    /* Initialize message headers */
    CFE_MSG_Init(CFE_MSG_PTR(STARNAV_AppData.HkTlm.TlmHeader), CFE_SB_ValueToMsgId(STARNAV_HK_TLM_MID),
                 sizeof(STARNAV_AppData.HkTlm));

    CFE_MSG_Init(CFE_MSG_PTR(STARNAV_AppData.AttitudeTlm.TlmHeader), CFE_SB_ValueToMsgId(STARNAV_ATTITUDE_MID),
                 sizeof(STARNAV_AppData.AttitudeTlm));

    /* Initialize UART device */
    status = STARNAV_InitDevice();
    if (status != CFE_SUCCESS)
        return status;

    CFE_EVS_SendEvent(STARNAV_INIT_INF_EID, CFE_EVS_EventType_INFORMATION, "STARNAV App Initialized");

    return CFE_SUCCESS;
}

/**
 * @brief Process Data from the Star Tracker Device
 *
 * Reads data from the connected star tracker device via UART and updates
 * telemetry accordingly. This function is called periodically from the main
 * loop during Software Bus receive timeouts.
 *
 * On successful read:
 * - Updates LastUpdateTime in housekeeping telemetry
 *
 * On read error (STARNAV_NO_DATA):
 * - Increments DeviceErrors counter in housekeeping telemetry
 *
 * @return None
 *
 * @note Called automatically during normal operation; no user intervention needed
 * @see STARNAV_ReadDevice, STARNAV_Main
 */
void STARNAV_ProcessDeviceData(void)
{
    CFE_Status_t status = STARNAV_ReadDevice();

    if (status == CFE_SUCCESS)
    {
        STARNAV_AppData.HkTlm.LastUpdateTime = CFE_TIME_GetTime().Seconds;
    }
    else if (status == STARNAV_NO_DATA)
    {
        STARNAV_AppData.HkTlm.DeviceErrors++;
    }
}

/**
 * @brief Process Commands Received via Software Bus
 *
 * Processes command and housekeeping request messages received from the
 * Software Bus. Extracts the message ID and dispatches to the appropriate
 * handler based on the message type.
 *
 * Currently supported message types:
 * - STARNAV_SEND_HK_MID: Timestamps and transmits housekeeping telemetry
 * - Unknown messages: Increments CommandErrorCounter
 *
 * @param[in] SBBufPtr Pointer to Software Bus message buffer containing
 *                     the received command or request
 *
 * @return None
 *
 * @note This function is called from STARNAV_Main when a message is received
 * @see STARNAV_Main
 */
void STARNAV_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t msg_id = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &msg_id);

    switch (CFE_SB_MsgIdToValue(msg_id))
    {
        case STARNAV_SEND_HK_MID:
            CFE_SB_TimeStampMsg(CFE_MSG_PTR(STARNAV_AppData.HkTlm.TlmHeader));
            CFE_SB_TransmitMsg(CFE_MSG_PTR(STARNAV_AppData.HkTlm.TlmHeader), true);
            break;

        default:
            STARNAV_AppData.HkTlm.CommandErrorCounter++;
            break;
    }
}
