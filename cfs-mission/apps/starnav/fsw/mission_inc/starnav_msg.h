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
 * @file starnav_msg.h
 *
 * @brief StarNav Message Definitions
 *
 * This header defines message structures for communication between the StarNav
 * CFS application and the STM32-based star tracker hardware via UART interface.
 *
 * The message structures include:
 * - Attitude telemetry with quaternion and Euler angle representations
 * - Device status and performance metrics
 * - Application housekeeping data
 * - Command message formats
 *
 * @note All telemetry message formats must maintain compatibility with the
 *       STM32 firmware UART packet protocol
 *
 * @author Robert Hamilton
 * @date 2025
 */

#ifndef STARNAV_MSG_H
#define STARNAV_MSG_H

/*
** Include Files
*/
#include "cfe.h"

#include <stdint.h>

/*
** Type Definitions
*/
/**
 * @brief Attitude Data Message
 *
 * @note Must match STM32 UART packet format
 */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader; /**< cFE telemetry header */

    float Quaternion[4]; /**< Attitude quaternion (w, x, y, z) */

    float EulerAngles[3]; /**< Euler angles in degrees (roll, pitch, yaw) */

    float Confidence; /**< Confidence level (0.0 to 1.0) */

    uint8_t NumStarsMatched; /**< Number of stars matched */

    uint8_t OperatingMode; /**< Operating mode: 0=idle, 1=tracking, 2=lost-in-space */

    uint32_t DeviceTimestamp; /**< Device timestamp in milliseconds */

    CFE_TIME_SysTime_t CFE_Timestamp; /**< cFE reception timestamp */

} STARNAV_AttitudeTlm_t;

/**
 * @brief Status Telemetry Message
 *
 * Contains operational status information including star tracking performance,
 * processing metrics, device uptime, and communication statistics
 */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader; /**< cFS telemetry header */

    uint8_t  SystemState;      /**< Current system state */
    uint8_t  StarsVisible;     /**< Number of stars visible in field of view */
    uint8_t  StarsMatched;     /**< Number of stars successfully matched to catalog */
    float    ProcessingTimeMs; /**< Image processing time in milliseconds */
    uint32_t DeviceUptime;     /**< Device uptime in seconds */
    uint16_t PacketsReceived;  /**< Total number of packets received */
    uint16_t PacketErrors;     /**< Total number of packet errors detected */

} STARNAV_StatusTlm_t;

/**
 * @brief Housekeeping Telemetry Message
 *
 * Contains application health and status data including command execution counters,
 * device communication error counts, and last update timestamp
 */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader; /**< cFS telemetry header */

    uint8_t  CommandCounter;      /**< Count of valid commands received */
    uint8_t  CommandErrorCounter; /**< Count of invalid commands received */
    uint16_t DeviceErrors;        /**< Number of device communication errors */
    uint32_t LastUpdateTime;      /**< Timestamp of last telemetry update in seconds */

} STARNAV_HkTlm_t;

/**
 * @brief No-Args Command
 *
 * Generic command message structure for commands that require no additional parameters
 * beyond the standard cFS command header
 */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader; /**< cFS command header */
} STARNAV_NoArgsCmd_t;

#endif /* STARNAV_MSG_H */
