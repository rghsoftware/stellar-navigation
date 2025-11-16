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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
 * @file starnav_device.c
 * @brief Star tracker device interface implementation
 *
 * This file implements the low-level UART communication interface for the
 * star tracker hardware device. It handles device initialization, UART
 * configuration, packet reception, parsing, and telemetry publishing.
 *
 * @par Protocol Overview
 * The star tracker communicates via UART using a simple packet protocol:
 * - Sync bytes: 0xAA 0x55
 * - Message ID: 1 byte
 * - Length: 1 byte (payload length)
 * - Payload: variable length
 * - CRC: 2 bytes
 *
 * @author Robert Hamilton
 * @date 2025
 */

/*
** Include Files:
*/
#define _DEFAULT_SOURCE  // Must be before includes
#include "starnav_app.h"

#include "cfe.h"
#include "common_types.h"
#include "osapi-error.h"
#include "osapi-file.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

/*
** Global Data
*/

/** @brief UART device path for star tracker communication */
#define STARNAV_UART_DEVICE "/dev/ttyAMA0"

/** @brief UART baud rate (115200 bps) */
#define STARNAV_UART_BAUD (B115200)

/** @brief Maximum packet size in bytes */
#define STARNAV_MAX_PACKET (128)

/* Packet framing */
/** @brief First sync byte for packet framing */
#define SYNC_BYTE_1 (0xAAU)

/** @brief Second sync byte for packet framing */
#define SYNC_BYTE_2 (0x55U)

/** @brief Message ID for attitude telemetry */
#define MSG_ATTITUDE (0x01U)

/** @brief Message ID for status telemetry */
#define MSG_STATUS (0x02U)

/*
** Functions
*/

/**
 * @brief Configure UART settings for Linux platform
 *
 * Applies termios configuration for 8N1 communication at 115200 baud.
 * Platform-specific implementation for Linux systems.
 *
 * @param[in] fd OSAL file descriptor for the UART device
 *
 * @return CFE_SUCCESS on successful configuration
 * @return CFE_STATUS_EXTERNAL_RESOURCE_FAIL if termios configuration fails
 */
CFE_Status_t STARNAV_ConfigureUART_Linux(osal_id_t fd);

/**
 * @brief Initialize the UART device connection to the star tracker
 *
 * Opens the UART device file and configures the serial port settings
 * appropriate for the star tracker hardware. The configuration is
 * platform-specific (currently supports Linux).
 *
 * @par Assumptions and Limitations
 * - UART device must exist at STARNAV_UART_DEVICE path
 * - Requires appropriate permissions to access the UART device
 * - Linux-specific termios configuration
 *
 * @return CFE_SUCCESS on successful initialization
 * @return CFE_STATUS_EXTERNAL_RESOURCE_FAIL if device open fails
 * @return CFE_STATUS_NOT_IMPLEMENTED on unsupported platforms
 *
 * @see STARNAV_ConfigureUART_Linux
 */
CFE_Status_t STARNAV_InitDevice(void)
{
    CFE_Status_t status = CFE_STATUS_EXTERNAL_RESOURCE_FAIL;

    /* Open UART device */
    status = OS_OpenCreate(&(STARNAV_AppData.DeviceFd),
                           STARNAV_UART_DEVICE,
                           OS_FILE_FLAG_NONE, /* Don't create/truncate */
                           OS_READ_WRITE);

    if (status != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(STARNAV_DEV_ERR_EID, CFE_EVS_EventType_ERROR, "Failed to open UART device: %d", (int)status);
        return status;
    }

#ifdef __linux__
    status = STARNAV_ConfigureUART_Linux(STARNAV_AppData.DeviceFd);
#else
    CFE_EVS_SendEvent(STARNAV_DEV_ERR_EID,
                      CFE_EVS_EventType_ERROR,
                      "STARNAV_ConfigureUART not implemented for this platform");
    status = CFE_STATUS_NOT_IMPLEMENTED;
#endif /* ifdef MACRO */

    if (status == CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(STARNAV_DEV_INF_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "UART device opened: %s",
                          STARNAV_UART_DEVICE);
    }

    return status;
}

/**
 * @brief Read data from the UART device
 *
 * Attempts to read available data from the UART interface and buffer it
 * for packet processing. Implements a state machine to detect packet sync
 * bytes, accumulate complete packets, and process them when received.
 *
 * @par Packet State Machine
 * The function maintains a static packet buffer and index to accumulate
 * data across multiple calls:
 * 1. Wait for SYNC_BYTE_1 (0xAA)
 * 2. Verify SYNC_BYTE_2 (0x55)
 * 3. Accumulate header (msg_id, length)
 * 4. Accumulate payload and CRC based on length field
 * 5. Process complete packet and reset state
 *
 * @par Thread Safety
 * Uses static variables for packet buffering. Not thread-safe.
 *
 * @return CFE_SUCCESS if data was read and processed successfully
 * @return STARNAV_NO_DATA if no data available on UART
 * @return CFE_STATUS_EXTERNAL_RESOURCE_FAIL on UART read error
 *
 * @see STARNAV_ProcessPacket
 */
CFE_Status_t STARNAV_ReadDevice(void)
{
    int32    bytes_read                 = 0;
    uint8_t  buffer[STARNAV_MAX_PACKET] = {0};
    uint8_t  length                     = 0;
    uint16_t total_length               = 0;
    int32    idx                        = 0;

    static int     packet_index                      = 0;
    static uint8_t packet_buffer[STARNAV_MAX_PACKET] = {0};

    bytes_read = OS_read(STARNAV_AppData.DeviceFd, buffer, sizeof(buffer));
    if (bytes_read < 0)
    {
        CFE_EVS_SendEvent(STARNAV_DEV_ERR_EID, CFE_EVS_EventType_ERROR, "UART read error: %d", (int)bytes_read);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    if (bytes_read == 0)
    {
        return STARNAV_NO_DATA;
    }

    for (idx = 0; idx < bytes_read; idx++)
    {
        if (packet_index == 0 && buffer[idx] == SYNC_BYTE_1)
        {
            packet_buffer[packet_index] = buffer[idx];
            ++packet_index;
        }
        else if (packet_index == 1)
        {
            if (buffer[idx] == SYNC_BYTE_2)
            {
                packet_buffer[packet_index] = buffer[idx];
                ++packet_index;
            }
            else
            {
                packet_index = 0;
            }
        }
        else if (packet_index >= 2)
        {
            packet_buffer[packet_index] = buffer[idx];
            ++packet_index;

            /* Check if we have the complete packet */
            if (packet_index >= 4)
            {
                length       = packet_buffer[3];
                total_length = 4 + length + 2;  // header + payload + CRC

                if (packet_index >= total_length)
                {
                    STARNAV_ProcessPacket(packet_buffer, total_length);
                    packet_index = 0;
                }
                else if (packet_index >= STARNAV_MAX_PACKET)
                {
                    packet_index = 0;  // Reset on overflow
                }
            }
        }
    }

    return CFE_SUCCESS;
}

/**
 * @brief Process a complete packet received from the star tracker device
 *
 * Parses the packet structure and extracts telemetry data based on the
 * message ID. Currently supports attitude data packets containing quaternion
 * and Euler angle representations. Publishes telemetry to the cFS Software Bus.
 *
 * @par Packet Format
 * - Byte 0-1: Sync bytes (0xAA 0x55)
 * - Byte 2: Message ID
 * - Byte 3: Payload length
 * - Byte 4+: Payload data
 * - Last 2 bytes: CRC (not yet verified)
 *
 * @par Attitude Packet Payload
 * - Bytes 0-15: Quaternion (4 floats: w, x, y, z)
 * - Bytes 16-27: Euler angles (3 floats: roll, pitch, yaw)
 *
 * @param[in] packet Pointer to the complete packet data buffer
 * @param[in] length Length of the packet data in bytes (currently unused)
 *
 * @note CRC verification is not yet implemented (TODO)
 * @note Updates STARNAV_AppData.StatusTlm.PacketsReceived counter
 *
 * @see MSG_ATTITUDE
 * @see MSG_STATUS
 */
void STARNAV_ProcessPacket(uint8_t *packet, uint16_t length)
{
    uint8_t  msg_id          = packet[2];
    uint8_t *payload         = &packet[4];
    size_t   quaternion_sz   = sizeof(float) * 4;
    size_t   euler_angles_sz = sizeof(float) * 3;

    /* TODO: Verify CRC */

    STARNAV_AppData.StatusTlm.PacketsReceived++;
    if (msg_id == MSG_ATTITUDE)
    {
        /* Parse attitude data */
        memcpy(&STARNAV_AppData.AttitudeTlm.Quaternion, payload, quaternion_sz);
        memcpy(&STARNAV_AppData.AttitudeTlm.EulerAngles, payload + quaternion_sz, euler_angles_sz);

        /* Add timestamp */
        STARNAV_AppData.AttitudeTlm.CFE_Timestamp = CFE_TIME_GetTime();

        /* Publish to Software Bus */
        CFE_SB_TimeStampMsg(CFE_MSG_PTR(STARNAV_AppData.AttitudeTlm.TlmHeader));
        CFE_SB_TransmitMsg(CFE_MSG_PTR(STARNAV_AppData.AttitudeTlm.TlmHeader), true);
    }
}

#ifdef __linux__

/**
 * @brief Configure UART settings for Linux platform (implementation)
 *
 * Configures the UART port using termios for raw binary communication:
 * - Baud rate: 115200 (both input and output)
 * - Data bits: 8
 * - Parity: None
 * - Stop bits: 1
 * - Flow control: None
 * - Mode: Raw (no line editing, no echo, no signals)
 *
 * @param[in] fd OSAL file descriptor for the UART device
 *
 * @return CFE_SUCCESS on successful configuration
 * @return CFE_STATUS_EXTERNAL_RESOURCE_FAIL if tcsetattr fails
 *
 * @note Only available on Linux platforms
 * @note Closes the file descriptor if configuration fails
 */
CFE_Status_t STARNAV_ConfigureUART_Linux(osal_id_t fd)
{
    struct termios options;
    int32          status = 0;

    memset(&options, 0, sizeof(options));

    /* Set baud rate for input and output */
    cfsetispeed(&options, STARNAV_UART_BAUD);
    cfsetospeed(&options, STARNAV_UART_BAUD);

    /* Configure control flags: 8N1, no flow control */
    options.c_cflag |= (CLOCAL | CREAD); /* Enable receiver, ignore modem control lines */
    options.c_cflag &= ~PARENB;          /* No parity */
    options.c_cflag &= ~CSTOPB;          /* 1 stop bit */
    options.c_cflag &= ~CSIZE;           /* Clear data size mask */
    options.c_cflag |= CS8;              /* 8 data bits */
    options.c_cflag &= ~CRTSCTS;         /* No hardware flow control */

    /* Configure local flags: raw mode */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Raw input, no echo, no signals */

    /* Configure input flags: disable software flow control */
    options.c_lflag &= ~(IXON | IXOFF | IXANY);

    /* Configure output flags: raw output */
    options.c_oflag &= ~OPOST;

    /* Apply settings immediately */
    status = tcsetattr(STARNAV_AppData.DeviceFd, TCSANOW, &options);
    if (status < 0)
    {
        CFE_EVS_SendEvent(STARNAV_DEV_ERR_EID,
                          CFE_EVS_EventType_ERROR,
                          "Failed to apply termios settings: %d",
                          (int)status);
        OS_close(STARNAV_AppData.DeviceFd);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    return CFE_SUCCESS;
}
#endif /* __linux__ */
