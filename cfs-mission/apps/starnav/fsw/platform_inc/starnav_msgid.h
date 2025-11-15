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
 * @file starnav_msgid.h
 *
 * Message IDs for STARNAV application.
 * These must be unique across all cFS apps.
 */

#ifndef STARNAV_MSGID_H
#define STARNAV_MSGID_H

/* Command Message IDs */
#define STARNAV_CMD_MID     (0x1880) /* Commands to STARNAV app */
#define STARNAV_SEND_HK_MID (0x1881) /* Request housekeeping */

/* Telemetry Message IDs */
#define STARNAV_HK_TLM_MID   (0x0880) /* Housekeeping telemetry */
#define STARNAV_ATTITUDE_MID (0x0881) /* Attitude data from STM32 */
#define STARNAV_STATUS_MID   (0x0882) /* Device status telemetry */

#endif /* STARNAV_MSGID_H */
