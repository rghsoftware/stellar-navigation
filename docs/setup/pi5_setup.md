# Raspberry Pi 5 Setup Guide for Stellar Navigation Demo

**What this does:** Transforms a fresh Pi 5 into a complete stellar navigation demonstration system running NASA's Core Flight System (cFS) with real-time 3D visualization and gimbal control.

**Time to complete:** 4-5 hours (can be done in stages)

---

## 🎯 Quick Start Overview

You'll build a distributed system where:
- **STM32** runs stellar navigation algorithms → sends attitude data over UART
- **Pi 5** runs cFS framework → processes data, controls servos, hosts dashboard
- **Web browser** displays 3D spacecraft visualization in real-time

**Prerequisites:**
- Raspberry Pi 5 (4GB+ RAM recommended)
- MicroSD card (32GB+ recommended)
- STM32F405 Black Pill (firmware ready)
- Network connection for Pi
- Basic Linux terminal familiarity

---

## 📋 Setup Phases

| Phase | Task | Time | Complexity |
|-------|------|------|------------|
| 1 | OS Installation & Config | 30 min | Easy |
| 2 | cFS Installation | 60 min | Medium |
| 3 | STARNAV_APP Creation | 90 min | Medium-Hard |
| 4 | Python Bridge & Flask | 45 min | Medium |
| 5 | UART & Servo Setup | 30 min | Easy |

---

## Phase 1: OS Installation & Configuration

**Goal:** Get Pi 5 running with proper configuration for embedded development.

### Step 1.1: Install Raspberry Pi OS

**Install Raspberry Pi OS Lite (64-bit)** using Raspberry Pi Imager:

1. **Download Raspberry Pi Imager** from https://www.raspberrypi.com/software/
2. **Select OS:** Choose "Raspberry Pi OS (64-bit)" - Lite version is fine
3. **Configure settings** (click gear icon):
   - Set hostname: `starnav-demo`
   - Enable SSH
   - Set username: `pi` (or your preference)
   - Set password
   - Configure WiFi (if using wireless)
   - Set locale and timezone
4. **Write to SD card**
5. **Boot Pi 5** with the SD card

> ⏱️ **Time:** 10-15 minutes for imaging and first boot

> ✅ **Expected result:** Pi boots, you can SSH into it: `ssh pi@starnav-demo.local`

### Step 1.2: Initial System Configuration

**Update system and install essential tools:**

```bash
# Update package lists and upgrade
sudo apt update && sudo apt upgrade -y

# Install essential build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-venv \
    minicom \
    screen \
    vim \
    htop

# Install development libraries
sudo apt install -y \
    libyaml-dev \
    libssl-dev \
    libffi-dev \
    python3-dev
```

> ⏱️ **Time:** 10-15 minutes (depends on internet speed)

> ✅ **Expected result:** All packages install without errors

### Step 1.3: Enable UART for STM32 Communication

**Configure serial port for data from STM32:**

1. **Edit boot config:**
   ```bash
   sudo nano /boot/firmware/config.txt
   ```

2. **Add these lines** at the end:
   ```
   # Enable UART for stellar navigation data
   enable_uart=1
   dtoverlay=disable-bt
   ```

3. **Save and exit** (Ctrl+X, Y, Enter)

4. **Disable serial console:**
   ```bash
   sudo raspi-config
   ```
   - Navigate to: `3 Interface Options` → `I6 Serial Port`
   - "Would you like a login shell accessible over serial?" → **No**
   - "Would you like the serial port hardware enabled?" → **Yes**
   - Exit and reboot: `sudo reboot`

5. **After reboot, verify UART is available:**
   ```bash
   ls -l /dev/ttyAMA0
   ```

> ✅ **Expected result:** You see `/dev/ttyAMA0` device file

> 💡 **Tip:** The Pi 5 uses `ttyAMA0` for UART (GPIO 14/15). This is where STM32 connects.

### Step 1.4: Set Up GPIO Permissions for Servo Control

**Install lgpio (Pi 5 compatible GPIO library):**

> 💡 **Important:** Pi 5 uses the RP1 chip for GPIO, so `pigpio` won't work. Use `lgpio` instead.

```bash
# Install lgpio library and Python bindings
sudo apt install -y python3-lgpio

# Install gpiozero (high-level library that uses lgpio on Pi 5)
sudo apt install -y python3-gpiozero

# Add user to gpio group for permissions
sudo usermod -a -G gpio pi

# Log out and back in for group changes to take effect
# Or run: newgrp gpio
```

> ✅ **Expected result:** Packages install successfully, no errors

---

## Phase 2: cFS Installation

**Goal:** Build and test NASA's Core Flight System on Pi 5.

### Step 2.1: Clone cFS Repository

**Get the official cFS code:**

```bash
# Create workspace directory
mkdir -p ~/workspace
cd ~/workspace

# Clone cFS bundle (includes cFE, OSAL, PSP, and sample apps)
git clone https://github.com/nasa/cFS.git
cd cFS

# Initialize submodules (this gets all the framework components)
git submodule update --init --recursive
```

> ⏱️ **Time:** 5 minutes

> ✅ **Expected result:** You have `~/workspace/cFS` directory with many subdirectories

### Step 2.2: Configure Build for Raspberry Pi

**Set up platform-specific configuration:**

```bash
cd ~/workspace/cFS

# Copy sample configuration as starting point
cp cfe/cmake/Makefile.sample Makefile
cp -r cfe/cmake/sample_defs sample_defs
```

**Edit the arch build configuration:**

```bash
nano sample_defs/targets.cmake
```

**Replace contents with:**

```cmake
# Raspberry Pi 5 ARM64 target configuration
list(APPEND MISSION_GLOBAL_APPLIST sample_app)

# Define the target for Raspberry Pi
list(APPEND MISSION_CPUNAMES cpu1)

set(cpu1_PROCESSORID 1)
set(cpu1_APPLIST sample_app)
set(cpu1_FILELIST cfe_es_startup.scr)
set(cpu1_SYSTEM default)

# Platform selection
set(cpu1_PSP pc-linux)

# Build options
set(ENABLE_UNIT_TESTS FALSE)
```

**Save and exit** (Ctrl+X, Y, Enter)

### Step 2.3: Build cFS

**Compile the flight software:**

```bash
cd ~/workspace/cFS

# Prepare build system
make prep

# Build cFS (uses all CPU cores)
make -j$(nproc)

# Install to build directory
make install
```

> ⏱️ **Time:** 30-45 minutes (first build is slow)

> ⚠️ **Warning:** If build fails with memory errors, use `make -j2` instead for fewer parallel jobs

> ✅ **Expected result:** Build completes with "Install the project..." message, no fatal errors

### Step 2.4: Test cFS Runs

**Verify the framework works:**

```bash
cd ~/workspace/cFS/build/exe/cpu1

# Run cFS
./core-cpu1
```

> ✅ **Expected result:** You see cFS startup messages like:
> ```
> CFE_PSP: Starting Up
> CFE_ES_Main: Started
> SAMPLE_APP: Initialized
> ```

**Stop cFS:** Press Ctrl+C

> 💡 **Tip:** If you see "CFE_PSP_Main: Starting Up" and no errors, cFS is working correctly!

---

## Phase 3: Creating STARNAV_APP

**Goal:** Build a custom cFS application that reads UART data from STM32 and publishes it to the Software Bus.

### Step 3.1: Create App Directory Structure

**Set up the application skeleton:**

```bash
cd ~/workspace/cFS/apps

# Create new app directory
mkdir -p starnav/fsw/{mission_inc,platform_inc,src}
mkdir -p starnav/fsw/tables

# Create necessary directories
mkdir -p starnav/fsw/src
```

### Step 3.2: Create Message ID Definitions

**Define unique message IDs for your app:**

```bash
nano ~/workspace/cFS/apps/starnav/fsw/platform_inc/starnav_msgids.h
```

**Add this content:**

```c
#ifndef STARNAV_MSGIDS_H
#define STARNAV_MSGIDS_H

/*
 * Message IDs for STARNAV application
 * These must be unique across all cFS apps
 */

/* Command Message IDs */
#define STARNAV_CMD_MID         0x1880  /* Commands to STARNAV app */
#define STARNAV_SEND_HK_MID     0x1881  /* Request housekeeping */

/* Telemetry Message IDs */
#define STARNAV_HK_TLM_MID      0x0880  /* Housekeeping telemetry */
#define STARNAV_ATTITUDE_MID    0x0881  /* Attitude data from STM32 */
#define STARNAV_STATUS_MID      0x0882  /* Device status telemetry */

#endif /* STARNAV_MSGIDS_H */
```

### Step 3.3: Create Message Structure Definitions

**Define data structures for messages:**

```bash
nano ~/workspace/cFS/apps/starnav/fsw/mission_inc/starnav_msg.h
```

**Add this content:**

```c
#ifndef STARNAV_MSG_H
#define STARNAV_MSG_H

#include "cfe.h"

/*
 * Attitude Data Message - matches STM32 UART packet format
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;
    
    /* Attitude as quaternion (w, x, y, z) */
    float Quaternion[4];
    
    /* Euler angles in degrees (roll, pitch, yaw) */
    float EulerAngles[3];
    
    /* Match quality/confidence (0.0 to 1.0) */
    float Confidence;
    
    /* Number of stars successfully matched */
    uint8_t NumStarsMatched;
    
    /* Current operating mode (0=idle, 1=tracking, 2=lost-in-space) */
    uint8_t OperatingMode;
    
    /* Timestamp from STM32 (milliseconds) */
    uint32_t DeviceTimestamp;
    
    /* Reception timestamp from cFE */
    CFE_TIME_SysTime_t CFE_Timestamp;
    
} STARNAV_AttitudeTlm_t;

/*
 * Device Status Message
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;
    
    uint8_t  SystemState;       /* 0=idle, 1=tracking, 2=searching */
    uint8_t  StarsVisible;      /* Total stars detected */
    uint8_t  StarsMatched;      /* Stars matched to catalog */
    float    ProcessingTimeMs;  /* Algorithm execution time */
    uint32_t DeviceUptime;      /* STM32 uptime in seconds */
    uint16_t PacketsReceived;   /* Total packets received */
    uint16_t PacketErrors;      /* CRC errors detected */
    
} STARNAV_StatusTlm_t;

/*
 * Housekeeping Message
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;
    
    uint8_t  CommandCounter;    /* Count of valid commands */
    uint8_t  CommandErrorCounter; /* Count of invalid commands */
    uint16_t DeviceErrors;      /* Device communication errors */
    uint32_t LastUpdateTime;    /* Time of last data from STM32 */
    
} STARNAV_HkTlm_t;

/*
 * Command Messages
 */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
} STARNAV_NoArgsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8_t  Mode;              /* Target operating mode */
    float    ExposureTimeMs;    /* Camera exposure setting */
    float    Threshold;         /* Star detection threshold */
} STARNAV_ModeCmd_t;

#endif /* STARNAV_MSG_H */
```

### Step 3.4: Create Main Application Code

**Implement the core application logic:**

```bash
nano ~/workspace/cFS/apps/starnav/fsw/src/starnav_app.c
```

> 📝 **Note:** This is a long file. I'll provide the essential structure.

**Add this content:**

```c
#include "starnav_app.h"
#include "starnav_msg.h"
#include "starnav_msgids.h"
#include "starnav_device.h"
#include "starnav_version.h"

#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/*
 * Global Data
 */
STARNAV_AppData_t STARNAV_AppData;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Application entry point and main process loop                  */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void STARNAV_Main(void)
{
    int32 status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
     * Create the first Performance ID marker
     */
    CFE_ES_PerfLogEntry(STARNAV_PERF_ID);

    /*
     * Perform application initialization
     */
    status = STARNAV_Init();
    if (status != CFE_SUCCESS)
    {
        STARNAV_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /*
     * Main app loop
     */
    while (CFE_ES_RunLoop(&STARNAV_AppData.RunStatus))
    {
        /* 
         * Performance marker - end of cycle
         */
        CFE_ES_PerfLogExit(STARNAV_PERF_ID);

        /* 
         * Pend on receipt of command packet with 100ms timeout
         */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr,
                                     STARNAV_AppData.CommandPipe,
                                     100);

        /*
         * Performance marker - start of next cycle
         */
        CFE_ES_PerfLogEntry(STARNAV_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            STARNAV_ProcessCommandPacket(SBBufPtr);
        }
        else if (status == CFE_SB_TIME_OUT)
        {
            /* 
             * Timeout is normal - use this time to read from UART device
             */
            STARNAV_ProcessDeviceData();
        }
        else
        {
            CFE_EVS_SendEvent(STARNAV_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                             "SB Pipe Read Error, RC = 0x%08X", (unsigned int)status);
            STARNAV_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    /*
     * Exit the application
     */
    CFE_ES_ExitApp(STARNAV_AppData.RunStatus);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialization                                                  */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 STARNAV_Init(void)
{
    int32 status;

    STARNAV_AppData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
     * Initialize app data
     */
    memset(&STARNAV_AppData, 0, sizeof(STARNAV_AppData));

    /*
     * Register the app with Executive Services
     */
    CFE_ES_RegisterApp();

    /*
     * Register the events
     */
    status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("STARNAV: Error Registering Events, RC = 0x%08X\n",
                            (unsigned int)status);
        return status;
    }

    /*
     * Create Software Bus message pipe
     */
    status = CFE_SB_CreatePipe(&STARNAV_AppData.CommandPipe,
                              STARNAV_PIPE_DEPTH,
                              "STARNAV_CMD_PIPE");
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(STARNAV_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                         "Error creating pipe, RC = 0x%08X", (unsigned int)status);
        return status;
    }

    /*
     * Subscribe to Housekeeping request commands
     */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(STARNAV_SEND_HK_MID),
                             STARNAV_AppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(STARNAV_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                         "Error subscribing to HK, RC = 0x%08X", (unsigned int)status);
        return status;
    }

    /*
     * Subscribe to ground command packets
     */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(STARNAV_CMD_MID),
                             STARNAV_AppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(STARNAV_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                         "Error subscribing to commands, RC = 0x%08X", (unsigned int)status);
        return status;
    }

    /*
     * Initialize message headers
     */
    CFE_MSG_Init(CFE_MSG_PTR(STARNAV_AppData.HkTlm.TlmHeader),
                CFE_SB_ValueToMsgId(STARNAV_HK_TLM_MID),
                sizeof(STARNAV_AppData.HkTlm));

    CFE_MSG_Init(CFE_MSG_PTR(STARNAV_AppData.AttitudeTlm.TlmHeader),
                CFE_SB_ValueToMsgId(STARNAV_ATTITUDE_MID),
                sizeof(STARNAV_AppData.AttitudeTlm));

    CFE_MSG_Init(CFE_MSG_PTR(STARNAV_AppData.StatusTlm.TlmHeader),
                CFE_SB_ValueToMsgId(STARNAV_STATUS_MID),
                sizeof(STARNAV_AppData.StatusTlm));

    /*
     * Initialize the UART device connection to STM32
     */
    status = STARNAV_InitDevice();
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(STARNAV_DEV_ERR_EID, CFE_EVS_EventType_ERROR,
                         "Error initializing device, RC = 0x%08X", (unsigned int)status);
        return status;
    }

    CFE_EVS_SendEvent(STARNAV_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                     "STARNAV App Initialized. Version %d.%d.%d.%d",
                     STARNAV_MAJOR_VERSION, STARNAV_MINOR_VERSION,
                     STARNAV_REVISION, STARNAV_MISSION_REV);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process device data from UART                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void STARNAV_ProcessDeviceData(void)
{
    int32 status;
    
    /* Read from UART device */
    status = STARNAV_ReadDevice();
    
    if (status == CFE_SUCCESS)
    {
        /* Data was successfully read and published to Software Bus */
        STARNAV_AppData.HkTlm.LastUpdateTime = CFE_TIME_GetTime().Seconds;
    }
    else if (status != STARNAV_NO_DATA)
    {
        /* Only count actual errors, not "no data available" */
        STARNAV_AppData.HkTlm.DeviceErrors++;
    }
}

/* Command processing functions would go here... */
/* See full implementation in repository */
```

> 💡 **Tip:** This is a simplified version. The complete code includes command handling, telemetry generation, and error recovery.

### Step 3.5: Create Device Driver for UART

**Implement the UART communication layer:**

```bash
nano ~/workspace/cFS/apps/starnav/fsw/src/starnav_device.c
```

**Add this content:**

```c
#include "starnav_device.h"
#include "starnav_app.h"
#include "starnav_msg.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

/* UART device path */
#define STARNAV_UART_DEVICE "/dev/ttyAMA0"
#define STARNAV_UART_BAUD   B115200

/* Packet framing */
#define STARNAV_SYNC_BYTE_1  0xAA
#define STARNAV_SYNC_BYTE_2  0x55

/* Message types */
#define STARNAV_MSG_ATTITUDE 0x01
#define STARNAV_MSG_STATUS   0x02

/* Maximum packet size */
#define STARNAV_MAX_PACKET  128

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize UART device                                          */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 STARNAV_InitDevice(void)
{
    struct termios options;

    /* Open the UART device */
    STARNAV_AppData.DeviceFd = open(STARNAV_UART_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if (STARNAV_AppData.DeviceFd < 0)
    {
        CFE_EVS_SendEvent(STARNAV_DEV_ERR_EID, CFE_EVS_EventType_ERROR,
                         "Failed to open UART device %s: %s",
                         STARNAV_UART_DEVICE, strerror(errno));
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    /* Get current options */
    if (tcgetattr(STARNAV_AppData.DeviceFd, &options) < 0)
    {
        CFE_EVS_SendEvent(STARNAV_DEV_ERR_EID, CFE_EVS_EventType_ERROR,
                         "Failed to get UART attributes: %s", strerror(errno));
        close(STARNAV_AppData.DeviceFd);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    /* Set baud rate */
    cfsetispeed(&options, STARNAV_UART_BAUD);
    cfsetospeed(&options, STARNAV_UART_BAUD);

    /* Configure for raw binary mode */
    options.c_cflag |= (CLOCAL | CREAD);    /* Enable receiver, local mode */
    options.c_cflag &= ~PARENB;             /* No parity */
    options.c_cflag &= ~CSTOPB;             /* 1 stop bit */
    options.c_cflag &= ~CSIZE;              /* Clear data bits */
    options.c_cflag |= CS8;                 /* 8 data bits */
    options.c_cflag &= ~CRTSCTS;            /* No hardware flow control */

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Raw input */
    options.c_iflag &= ~(IXON | IXOFF | IXANY);          /* No software flow control */
    options.c_oflag &= ~OPOST;                            /* Raw output */

    /* Apply settings */
    if (tcsetattr(STARNAV_AppData.DeviceFd, TCSANOW, &options) < 0)
    {
        CFE_EVS_SendEvent(STARNAV_DEV_ERR_EID, CFE_EVS_EventType_ERROR,
                         "Failed to set UART attributes: %s", strerror(errno));
        close(STARNAV_AppData.DeviceFd);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    CFE_EVS_SendEvent(STARNAV_DEV_INF_EID, CFE_EVS_EventType_INFORMATION,
                     "UART device %s opened successfully", STARNAV_UART_DEVICE);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Read and process data from UART                                 */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 STARNAV_ReadDevice(void)
{
    uint8_t buffer[STARNAV_MAX_PACKET];
    ssize_t bytes_read;
    static uint8_t packet_buffer[STARNAV_MAX_PACKET];
    static int packet_index = 0;

    /* Read available data */
    bytes_read = read(STARNAV_AppData.DeviceFd, buffer, sizeof(buffer));

    if (bytes_read < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            CFE_EVS_SendEvent(STARNAV_DEV_ERR_EID, CFE_EVS_EventType_ERROR,
                             "UART read error: %s", strerror(errno));
            return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
        }
        return STARNAV_NO_DATA;
    }
    
    if (bytes_read == 0)
    {
        return STARNAV_NO_DATA;
    }

    /* Process bytes looking for packet frames */
    for (int i = 0; i < bytes_read; i++)
    {
        if (packet_index == 0 && buffer[i] == STARNAV_SYNC_BYTE_1)
        {
            packet_buffer[packet_index++] = buffer[i];
        }
        else if (packet_index == 1)
        {
            if (buffer[i] == STARNAV_SYNC_BYTE_2)
            {
                packet_buffer[packet_index++] = buffer[i];
            }
            else
            {
                packet_index = 0;  /* Reset if second sync byte wrong */
            }
        }
        else if (packet_index >= 2)
        {
            packet_buffer[packet_index++] = buffer[i];
            
            /* Check if we have message type and length */
            if (packet_index >= 4)
            {
                uint8_t msg_id = packet_buffer[2];
                uint8_t length = packet_buffer[3];
                uint16_t total_length = 4 + length + 2;  /* header + payload + CRC */
                
                if (packet_index >= total_length)
                {
                    /* Complete packet received - process it */
                    STARNAV_ProcessPacket(packet_buffer, total_length);
                    packet_index = 0;
                }
                else if (packet_index >= STARNAV_MAX_PACKET)
                {
                    /* Packet too large - reset */
                    packet_index = 0;
                }
            }
        }
    }

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process complete packet                                         */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void STARNAV_ProcessPacket(uint8_t *packet, uint16_t length)
{
    uint8_t msg_id = packet[2];
    uint8_t payload_length = packet[3];
    uint8_t *payload = &packet[4];
    
    /* TODO: Verify CRC */
    
    /* Update packet counter */
    STARNAV_AppData.StatusTlm.PacketsReceived++;
    
    switch (msg_id)
    {
        case STARNAV_MSG_ATTITUDE:
            STARNAV_ProcessAttitudePacket(payload, payload_length);
            break;
            
        case STARNAV_MSG_STATUS:
            STARNAV_ProcessStatusPacket(payload, payload_length);
            break;
            
        default:
            CFE_EVS_SendEvent(STARNAV_DEV_ERR_EID, CFE_EVS_EventType_ERROR,
                             "Unknown message type: 0x%02X", msg_id);
            break;
    }
}

/* Packet processing functions would go here... */
```

### Step 3.6: Create CMakeLists.txt

**Configure the build system for your app:**

```bash
nano ~/workspace/cFS/apps/starnav/CMakeLists.txt
```

**Add this content:**

```cmake
cmake_minimum_required(VERSION 3.5)
project(CFS_STARNAV C)

# Source files
set(APP_SRC_FILES
    fsw/src/starnav_app.c
    fsw/src/starnav_device.c
)

# Create the app module
add_cfe_app(starnav ${APP_SRC_FILES})

# Include directories
target_include_directories(starnav PUBLIC
    fsw/mission_inc
    fsw/platform_inc
)
```

### Step 3.7: Add App to Build Configuration

**Tell cFS to include your new app:**

```bash
cd ~/workspace/cFS
nano sample_defs/targets.cmake
```

**Modify to include starnav:**

```cmake
# Add starnav to global app list
list(APPEND MISSION_GLOBAL_APPLIST sample_app starnav)

# Define the target for Raspberry Pi
list(APPEND MISSION_CPUNAMES cpu1)

set(cpu1_PROCESSORID 1)
set(cpu1_APPLIST sample_app starnav)  # Add starnav here
set(cpu1_FILELIST cfe_es_startup.scr)
set(cpu1_SYSTEM default)
set(cpu1_PSP pc-linux)

set(ENABLE_UNIT_TESTS FALSE)
```

### Step 3.8: Configure Startup Script

**Tell cFS to start your app on boot:**

```bash
nano ~/workspace/cFS/build/exe/cpu1/cfe_es_startup.scr
```

**Add this line after the existing apps:**

```
CFE_APP, starnav,     STARNAV_Main,    STARNAV,   50,  16384, 0x0, 0;
```

> 💡 **Format:** `CFE_APP, name, entry_point, file, priority, stack_size, exception_action, unknown`

### Step 3.9: Rebuild cFS with STARNAV_APP

**Compile everything:**

```bash
cd ~/workspace/cFS

# Clean previous build
make distclean

# Reconfigure
make prep

# Build with your new app
make -j$(nproc)

# Install
make install
```

> ⏱️ **Time:** 30-45 minutes

> ✅ **Expected result:** Build completes without errors

**Test your app:**

```bash
cd ~/workspace/cFS/build/exe/cpu1
./core-cpu1
```

> ✅ **Expected result:** You see "STARNAV App Initialized" in the output

---

## Phase 4: Python Bridge & Flask Dashboard

**Goal:** Create the middleware that connects cFS to the web dashboard and servo control.

### Step 4.1: Create Python Virtual Environment

**Set up isolated Python environment:**

```bash
cd ~/workspace
python3 -m venv starnav-env
source starnav-env/bin/activate

# Install required packages
pip install --upgrade pip
pip install flask flask-socketio python-socketio gpiozero lgpio
```

> ✅ **Expected result:** All packages install successfully

### Step 4.2: Create Project Structure

**Set up the Python application:**

```bash
mkdir -p ~/workspace/starnav-dashboard/{static,templates}
cd ~/workspace/starnav-dashboard
```

### Step 4.3: Create cFS Bridge Service

**This reads from cFS Software Bus and publishes to WebSocket:**

```bash
nano ~/workspace/starnav-dashboard/cfs_bridge.py
```

**Add this content:**

```python
#!/usr/bin/env python3
"""
cFS Bridge Service
Reads telemetry from cFS Software Bus and publishes to WebSocket
"""

import time
import struct
import socket
import json
from flask import Flask
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'stellar-nav-secret'
socketio = SocketIO(app, cors_allowed_origins="*")

class CFSBridge:
    """Bridge between cFS and WebSocket clients"""
    
    def __init__(self):
        self.running = False
        self.latest_attitude = None
        self.latest_status = None
    
    def start(self):
        """Start reading from cFS"""
        self.running = True
        print("cFS Bridge started")
        
        # TODO: Implement actual cFS Software Bus reading
        # For now, generate dummy data for testing
        self._generate_test_data()
    
    def _generate_test_data(self):
        """Generate test data while we develop"""
        import math
        
        t = 0
        while self.running:
            # Simulate spacecraft rotation
            roll = 10 * math.sin(t * 0.1)
            pitch = 15 * math.cos(t * 0.15)
            yaw = 20 * math.sin(t * 0.05)
            
            attitude_data = {
                'quaternion': [0.9, 0.1, 0.2, 0.3],  # w, x, y, z
                'euler': [roll, pitch, yaw],
                'confidence': 0.95,
                'stars_matched': 8,
                'mode': 1,
                'timestamp': time.time()
            }
            
            # Emit to all connected clients
            socketio.emit('attitude_update', attitude_data)
            
            t += 0.1
            time.sleep(0.1)  # 10 Hz update rate

# Global bridge instance
bridge = CFSBridge()

@app.route('/')
def index():
    """Serve the dashboard"""
    from flask import render_template
    return render_template('dashboard.html')

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print('Client connected')
    emit('status', {'connected': True})

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print('Client disconnected')

if __name__ == '__main__':
    # Start bridge in background thread
    import threading
    bridge_thread = threading.Thread(target=bridge.start, daemon=True)
    bridge_thread.start()
    
    # Run Flask app
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
```

> 💡 **Tip:** This starts with test data. We'll integrate real cFS telemetry later.

### Step 4.4: Create Web Dashboard

**Build the 3D visualization interface:**

```bash
nano ~/workspace/starnav-dashboard/templates/dashboard.html
```

**Add this content:**

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Stellar Navigation Dashboard</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #0a0e27;
            color: #e0e0e0;
            overflow: hidden;
        }
        
        #container {
            display: grid;
            grid-template-columns: 300px 1fr;
            grid-template-rows: 60px 1fr;
            height: 100vh;
            gap: 10px;
            padding: 10px;
        }
        
        #header {
            grid-column: 1 / -1;
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            border-radius: 8px;
            padding: 15px 25px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        #header h1 {
            font-size: 24px;
            font-weight: 600;
        }
        
        .status-badge {
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: bold;
        }
        
        .status-connected {
            background: #2ecc71;
            color: #fff;
        }
        
        #sidebar {
            background: #1a1f3a;
            border-radius: 8px;
            padding: 20px;
            overflow-y: auto;
        }
        
        .telemetry-section {
            margin-bottom: 25px;
        }
        
        .telemetry-section h3 {
            font-size: 14px;
            color: #64b5f6;
            margin-bottom: 10px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .telemetry-value {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #2a3050;
        }
        
        .telemetry-value:last-child {
            border-bottom: none;
        }
        
        .telemetry-label {
            color: #9e9e9e;
            font-size: 13px;
        }
        
        .telemetry-data {
            font-weight: 600;
            color: #e0e0e0;
            font-size: 14px;
        }
        
        #viewer {
            background: #0f1428;
            border-radius: 8px;
            position: relative;
            overflow: hidden;
        }
        
        #canvas {
            width: 100%;
            height: 100%;
        }
    </style>
    
    <!-- Three.js for 3D rendering -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    
    <!-- Socket.IO for WebSocket -->
    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
</head>
<body>
    <div id="container">
        <div id="header">
            <h1>🛰️ Stellar Navigation System</h1>
            <span class="status-badge status-connected" id="status">CONNECTED</span>
        </div>
        
        <div id="sidebar">
            <div class="telemetry-section">
                <h3>Attitude</h3>
                <div class="telemetry-value">
                    <span class="telemetry-label">Roll</span>
                    <span class="telemetry-data" id="roll">0.0°</span>
                </div>
                <div class="telemetry-value">
                    <span class="telemetry-label">Pitch</span>
                    <span class="telemetry-data" id="pitch">0.0°</span>
                </div>
                <div class="telemetry-value">
                    <span class="telemetry-label">Yaw</span>
                    <span class="telemetry-data" id="yaw">0.0°</span>
                </div>
            </div>
            
            <div class="telemetry-section">
                <h3>Star Tracker</h3>
                <div class="telemetry-value">
                    <span class="telemetry-label">Stars Matched</span>
                    <span class="telemetry-data" id="stars">0</span>
                </div>
                <div class="telemetry-value">
                    <span class="telemetry-label">Confidence</span>
                    <span class="telemetry-data" id="confidence">0.0%</span>
                </div>
                <div class="telemetry-value">
                    <span class="telemetry-label">Mode</span>
                    <span class="telemetry-data" id="mode">IDLE</span>
                </div>
            </div>
        </div>
        
        <div id="viewer">
            <canvas id="canvas"></canvas>
        </div>
    </div>

    <script>
        // WebSocket connection
        const socket = io();
        
        socket.on('connect', () => {
            console.log('Connected to server');
            document.getElementById('status').textContent = 'CONNECTED';
        });
        
        socket.on('disconnect', () => {
            console.log('Disconnected from server');
            document.getElementById('status').textContent = 'DISCONNECTED';
        });
        
        socket.on('attitude_update', (data) => {
            updateTelemetry(data);
            updateSpacecraft(data);
        });
        
        // Update telemetry display
        function updateTelemetry(data) {
            document.getElementById('roll').textContent = data.euler[0].toFixed(1) + '°';
            document.getElementById('pitch').textContent = data.euler[1].toFixed(1) + '°';
            document.getElementById('yaw').textContent = data.euler[2].toFixed(1) + '°';
            document.getElementById('stars').textContent = data.stars_matched;
            document.getElementById('confidence').textContent = (data.confidence * 100).toFixed(1) + '%';
            
            const modes = ['IDLE', 'TRACKING', 'LOST-IN-SPACE'];
            document.getElementById('mode').textContent = modes[data.mode] || 'UNKNOWN';
        }
        
        // Three.js scene setup
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x0f1428);
        
        const canvas = document.getElementById('canvas');
        const camera = new THREE.PerspectiveCamera(45, canvas.clientWidth / canvas.clientHeight, 0.1, 1000);
        camera.position.set(5, 3, 5);
        camera.lookAt(0, 0, 0);
        
        const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
        renderer.setSize(canvas.clientWidth, canvas.clientHeight);
        
        // Add lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 5, 5);
        scene.add(directionalLight);
        
        // Create spacecraft (simple box for now)
        const geometry = new THREE.BoxGeometry(1, 0.5, 2);
        const material = new THREE.MeshStandardMaterial({ color: 0x4fc3f7 });
        const spacecraft = new THREE.Mesh(geometry, material);
        scene.add(spacecraft);
        
        // Add reference axes
        const axesHelper = new THREE.AxesHelper(2);
        scene.add(axesHelper);
        
        // Update spacecraft orientation
        function updateSpacecraft(data) {
            const [roll, pitch, yaw] = data.euler.map(d => d * Math.PI / 180);
            spacecraft.rotation.set(pitch, yaw, roll);
        }
        
        // Animation loop
        function animate() {
            requestAnimationFrame(animate);
            renderer.render(scene, camera);
        }
        animate();
        
        // Handle window resize
        window.addEventListener('resize', () => {
            const width = canvas.clientWidth;
            const height = canvas.clientHeight;
            camera.aspect = width / height;
            camera.updateProjectionMatrix();
            renderer.setSize(width, height);
        });
    </script>
</body>
</html>
```

### Step 4.5: Test the Dashboard

**Run the Flask server:**

```bash
cd ~/workspace/starnav-dashboard
source ~/workspace/starnav-env/bin/activate
python3 cfs_bridge.py
```

> ✅ **Expected result:** Server starts on `http://0.0.0.0:5000`

**Open browser on Pi** (or from another computer on same network):
```
http://starnav-demo.local:5000
```

> ✅ **Expected result:** You see the dashboard with a rotating spacecraft!

---

## Phase 5: UART & Servo Integration

**Goal:** Connect STM32 UART and set up servo gimbal control.

### Step 5.1: Verify UART Connection

**Test physical connection to STM32:**

1. **Connect wiring:**
   - STM32 TX (PA9) → Pi RX (GPIO 15)
   - STM32 RX (PA10) → Pi TX (GPIO 14)
   - STM32 GND → Pi GND

2. **Test with minicom:**
   ```bash
   minicom -D /dev/ttyAMA0 -b 115200
   ```

3. **Power on STM32** - you should see binary data streaming

> ✅ **Expected result:** Characters appear when STM32 transmits

### Step 5.2: Create Servo Control Module

**Add servo control to Python bridge:**

```bash
nano ~/workspace/starnav-dashboard/servo_control.py
```

**Add this content:**

```python
#!/usr/bin/env python3
"""
Servo Gimbal Control - Pi 5 Compatible
Translates spacecraft attitude to servo positions using gpiozero
"""

from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory
import time
import math

# Use lgpio factory for Pi 5 compatibility
factory = LGPIOFactory()

class ServoGimbal:
    """2-axis gimbal control using gpiozero with lgpio backend"""
    
    # GPIO pins for servos
    PAN_PIN = 17   # GPIO 17 for pan (yaw)
    TILT_PIN = 27  # GPIO 27 for tilt (pitch)
    
    # Mechanical limits (degrees)
    PAN_LIMIT = 90    # ±90° for pan
    TILT_LIMIT = 60   # ±60° for tilt
    
    def __init__(self):
        """Initialize servos using gpiozero"""
        # Create servo objects with extended pulse width range for better control
        # min_pulse_width and max_pulse_width in seconds
        self.pan_servo = Servo(
            self.PAN_PIN,
            min_pulse_width=0.5/1000,   # 0.5ms (500µs)
            max_pulse_width=2.5/1000,   # 2.5ms (2500µs)
            pin_factory=factory
        )
        
        self.tilt_servo = Servo(
            self.TILT_PIN,
            min_pulse_width=0.5/1000,
            max_pulse_width=2.5/1000,
            pin_factory=factory
        )
        
        # Center servos on startup
        self.center()
        print("Servo gimbal initialized (Pi 5 / lgpio)")
    
    def center(self):
        """Move servos to center position"""
        self.pan_servo.mid()
        self.tilt_servo.mid()
    
    def set_attitude(self, roll, pitch, yaw):
        """
        Set gimbal position based on spacecraft attitude
        
        Args:
            roll: Roll angle in degrees (not used for 2-axis gimbal)
            pitch: Pitch angle in degrees (controls tilt servo)
            yaw: Yaw angle in degrees (controls pan servo)
        """
        # Clamp angles to mechanical limits
        yaw = max(-self.PAN_LIMIT, min(self.PAN_LIMIT, yaw))
        pitch = max(-self.TILT_LIMIT, min(self.TILT_LIMIT, pitch))
        
        # Convert angles to servo values (-1 to +1)
        pan_value = yaw / self.PAN_LIMIT
        tilt_value = pitch / self.TILT_LIMIT
        
        # Set servo positions
        # gpiozero Servo uses -1 (min) to +1 (max), with 0 as center
        self.pan_servo.value = pan_value
        self.tilt_servo.value = tilt_value
    
    def cleanup(self):
        """Clean up resources"""
        self.center()
        time.sleep(0.5)
        self.pan_servo.close()
        self.tilt_servo.close()

# Test code
if __name__ == '__main__':
    print("Pi 5 Servo Test - Using gpiozero with lgpio")
    gimbal = ServoGimbal()
    
    try:
        # Demo pattern - smooth sweep
        print("\nTesting gimbal movement...")
        print("Sweeping through angles...")
        
        for angle in range(-45, 46, 5):
            gimbal.set_attitude(0, angle, angle)
            print(f"Position: Pitch={angle}°, Yaw={angle}°")
            time.sleep(0.2)
        
        print("\nReturning to center...")
        gimbal.center()
        time.sleep(1)
        
        print("\nTesting individual axes...")
        
        # Test pan only
        print("Pan test (yaw axis)...")
        for yaw in [-45, 0, 45, 0]:
            gimbal.set_attitude(0, 0, yaw)
            print(f"  Yaw: {yaw}°")
            time.sleep(0.5)
        
        # Test tilt only
        print("Tilt test (pitch axis)...")
        for pitch in [-30, 0, 30, 0]:
            gimbal.set_attitude(0, pitch, 0)
            print(f"  Pitch: {pitch}°")
            time.sleep(0.5)
        
        gimbal.center()
        print("\nTest complete - servos centered")
        
    except KeyboardInterrupt:
        print("\n\nStopping...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        gimbal.cleanup()
        print("Cleanup complete")
```

### Step 5.3: Test Servo Control

**Verify servos work:**

```bash
cd ~/workspace/starnav-dashboard
source ~/workspace/starnav-env/bin/activate
python3 servo_control.py
```

> ✅ **Expected result:** Servos move through test pattern

> ⚠️ **Warning:** Make sure servos have separate 6V power supply, NOT from Pi GPIO!

### Step 5.4: Integrate Servos with Bridge

**Update cfs_bridge.py to control servos:**

```bash
nano ~/workspace/starnav-dashboard/cfs_bridge.py
```

**Add servo import and integration:**

```python
# Add at top of file
from servo_control import ServoGimbal

class CFSBridge:
    def __init__(self):
        self.running = False
        self.latest_attitude = None
        self.latest_status = None
        
        # Initialize servo gimbal
        try:
            self.gimbal = ServoGimbal()
            print("Servo gimbal ready")
        except Exception as e:
            print(f"Servo initialization failed: {e}")
            self.gimbal = None
    
    def _generate_test_data(self):
        """Generate test data and control servos"""
        import math
        
        t = 0
        while self.running:
            roll = 10 * math.sin(t * 0.1)
            pitch = 15 * math.cos(t * 0.15)
            yaw = 20 * math.sin(t * 0.05)
            
            # Update gimbal
            if self.gimbal:
                self.gimbal.set_attitude(roll, pitch, yaw)
            
            attitude_data = {
                'quaternion': [0.9, 0.1, 0.2, 0.3],
                'euler': [roll, pitch, yaw],
                'confidence': 0.95,
                'stars_matched': 8,
                'mode': 1,
                'timestamp': time.time()
            }
            
            socketio.emit('attitude_update', attitude_data)
            
            t += 0.1
            time.sleep(0.1)
```

---

## 🎉 Testing the Complete System

### Full System Test

1. **Start cFS:**
   ```bash
   cd ~/workspace/cFS/build/exe/cpu1
   ./core-cpu1
   ```

2. **In another terminal, start dashboard:**
   ```bash
   cd ~/workspace/starnav-dashboard
   source ~/workspace/starnav-env/bin/activate
   python3 cfs_bridge.py
   ```

3. **In another terminal, send test data from STM32:**
   - Power on STM32
   - It should start sending attitude packets

4. **Open browser:** `http://starnav-demo.local:5000`

> ✅ **Expected result:** 
> - Dashboard shows real-time 3D spacecraft
> - Telemetry values update
> - Gimbal moves to match spacecraft attitude
> - cFS logs show "STARNAV: Attitude received"

---

## 📚 Next Steps

Now that your system is running:

### Immediate Improvements
- [ ] Connect real STM32 UART data (replace test data)
- [ ] Add CRC verification for UART packets
- [ ] Implement command path (browser → cFS → STM32)
- [ ] Add error handling and recovery

### Enhanced Features
- [ ] Record telemetry to log files
- [ ] Add performance graphs (Chart.js)
- [ ] Implement mode control (tracking vs lost-in-space)
- [ ] Add star field visualization
- [ ] Create calibration routine for servos

### Production Hardening
- [ ] Create systemd services for auto-start
- [ ] Add watchdog timers
- [ ] Implement proper logging
- [ ] Add remote access capabilities
- [ ] Create backup/restore procedures

---

## 🐛 Troubleshooting

### cFS won't build
**Symptom:** CMake errors or compilation failures

**Solutions:**
1. Check you have all dependencies: `sudo apt install build-essential cmake git`
2. Try clean rebuild: `make distclean && make prep && make`
3. Check RAM usage - Pi 5 with 4GB should be fine, 2GB might struggle
4. Use fewer parallel jobs: `make -j2` instead of `make -j$(nproc)`

### UART not working
**Symptom:** `/dev/ttyAMA0` not found or permission denied

**Solutions:**
1. Verify UART enabled: Check `/boot/firmware/config.txt` has `enable_uart=1`
2. Check serial console disabled in `raspi-config`
3. Add user to dialout group: `sudo usermod -a -G dialout pi`
4. Reboot: `sudo reboot`
5. Test with loopback: Connect TX to RX and echo data

### Servos not moving
**Symptom:** No servo movement or jittery motion

**Solutions:**
1. Verify lgpio is installed: `python3 -c "import lgpio; print('lgpio OK')"`
2. Check gpiozero works: `python3 -c "from gpiozero import Servo; print('gpiozero OK')"`
3. Verify separate power supply (6V, 3A minimum)
4. Test with simple script: `python3 servo_control.py`
5. Check GPIO permissions: User should be in `gpio` group
6. Verify wiring: PWM signal must go to servo signal pin (usually orange/white wire)
7. Try different GPIO pins if hardware PWM not working

### Dashboard not loading
**Symptom:** Browser shows connection error

**Solutions:**
1. Check Flask is running: `ps aux | grep python3`
2. Verify network connection: `ping starnav-demo.local`
3. Check firewall: `sudo ufw status` (should be inactive by default)
4. Try IP address instead of hostname
5. Check Flask logs for errors

### cFS STARNAV_APP crashes
**Symptom:** cFS exits or STARNAV app shows errors

**Solutions:**
1. Check UART device exists: `ls -l /dev/ttyAMA0`
2. Verify message IDs don't conflict with other apps
3. Check event logs in cFS output
4. Increase stack size in startup script if stack overflow
5. Add debug prints to narrow down issue

---

## 📖 Reference

### Useful Commands

```bash
# Check cFS build status
cd ~/workspace/cFS
make -j$(nproc) 2>&1 | tee build.log

# Monitor UART traffic
minicom -D /dev/ttyAMA0 -b 115200

# Check Python virtual environment
source ~/workspace/starnav-env/bin/activate
pip list

# Test GPIO servo control (Python)
python3 -c "from gpiozero import Servo; s = Servo(17); s.mid(); import time; time.sleep(1)"

# Verify lgpio installation
python3 -c "import lgpio; print(f'lgpio version: {lgpio.version()}')"

# Monitor system resources
htop

# Check network connectivity
ip addr show
ping starnav-demo.local
```

### File Locations

| Component | Location |
|-----------|----------|
| cFS source | `~/workspace/cFS` |
| cFS build output | `~/workspace/cFS/build/exe/cpu1` |
| STARNAV app | `~/workspace/cFS/apps/starnav` |
| Python dashboard | `~/workspace/starnav-dashboard` |
| Virtual environment | `~/workspace/starnav-env` |
| UART device | `/dev/ttyAMA0` |
| Boot config | `/boot/firmware/config.txt` |

### Key Configuration Files

| File | Purpose |
|------|---------|
| `targets.cmake` | cFS build configuration |
| `cfe_es_startup.scr` | cFS app startup script |
| `starnav_msgids.h` | Message ID definitions |
| `config.txt` | Pi boot configuration |
| `cfs_bridge.py` | Python middleware |

---

## 🎓 Understanding the Architecture

**Data flow:**
```
STM32 (Stellar Nav) 
    → UART binary packets 
        → STARNAV_APP (cFS) 
            → Software Bus messages 
                → Python Bridge 
                    → WebSocket 
                        → Browser Dashboard
                    → Servo Control 
                        → Physical Gimbal
```

**Why this architecture:**
- **STM32:** Real-time algorithms need deterministic hardware
- **cFS:** Flight-proven framework, learning real spacecraft software
- **Python Bridge:** Rapid development for dashboard and servo control
- **WebSocket:** Real-time updates without polling
- **Servos:** Physical demonstration of attitude determination

**Benefits:**
- ✅ Mirrors real spacecraft architecture (distributed processing)
- ✅ Learns actual NASA flight software framework
- ✅ Separates concerns (algorithms vs UI vs control)
- ✅ Easy to extend with new features
- ✅ Demonstrates full system integration

---

**Total time:** ~4-5 hours for complete setup
**Result:** Fully functional stellar navigation demonstration system!
