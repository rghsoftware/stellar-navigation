# Raspberry Pi 5 Component Guide

**Component**: Raspberry Pi 5 Single-Board Computer  
**Role**: Flight software host, middleware, and user interface  
**Services**: cFS framework, Python bridge, web dashboard

---

## Overview

The Raspberry Pi 5 acts as the system integrator, running NASA's Core Flight System (cFS), providing a web-based
dashboard, and controlling the physical gimbal demonstration.

**Responsibilities:**

- Run cFS framework with STARNAV app
- Bridge UART data to Software Bus
- Serve real-time web dashboard
- Control servo gimbal via GPIO

---

## Hardware Selection

### Raspberry Pi 5 Models

**4GB RAM** ($60):

- Sufficient for this project
- Runs cFS + Python comfortably
- **Recommended** for budget builds

**8GB RAM** ($80):

- Future-proof for additional apps
- Better for development/debugging
- Recommended if expanding project

### Required Accessories

**Power Supply:**

- Official Raspberry Pi 27W USB-C PSU ($15)
- Or quality USB-C PD charger (27W+)
- ⚠️ Underpowered supplies cause instability

**Storage:**

- MicroSD card: 32GB minimum, Class 10/U3
- SanDisk Ultra or Samsung EVO recommended
- 64GB if recording telemetry logs

**Cooling:**

- Pi 5 runs hot under load
- Active cooling (fan) recommended
- Heatsink minimum requirement
- Target: <70°C under load

**Optional:**

- GPIO breakout cable for prototyping
- HDMI micro cable for initial setup
- Case with fan mount

---

## OS Installation

### Raspberry Pi OS Setup

**Step 1: Download Raspberry Pi Imager**

- URL: https://www.raspberrypi.com/software/
- Available for Windows, macOS, Linux

**Step 2: Configure Image**

```
1. Select OS: "Raspberry Pi OS (64-bit)"
   - Lite version acceptable (no desktop needed)
   - Full version if you want GUI

2. Click Settings (gear icon):
   - Hostname: starnav-fc
   - Enable SSH: ✓
   - Username: pi (or your choice)
   - Password: [set secure password]
   - WiFi: [configure if using wireless]
   - Timezone: [your location]

3. Write to SD card
```

**Step 3: First Boot**

```bash
# Insert SD card into Pi 5
# Connect power (green LED should flash)
# Wait 60 seconds for first boot

# SSH from another computer:
ssh pi@starnav-fc.local

# If .local doesn't resolve, find IP:
# Check your router's DHCP table
# Or use: nmap -sn 192.168.1.0/24
```

> ⚠️ **Note**: `.local` (mDNS) requires Avahi. Install during initial setup below.

---

## System Configuration

### Initial Update

```bash
# Update package database
sudo apt update

# Upgrade all packages
sudo apt upgrade -y

# Reboot
sudo reboot
```

⏱️ **Time**: 10-15 minutes

### Install mDNS Support

```bash
# Install Avahi for .local hostname resolution
sudo apt install -y avahi-daemon avahi-utils

# Enable and start service
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon

# Verify hostname is advertised
avahi-browse -a -t | grep starnav-fc
```

**What this does:**

- Enables `.local` hostname access (e.g., `starnav-fc.local`)
- Allows SSH without needing to find IP address
- Required for Raspberry Pi OS Lite (pre-installed on Desktop edition)

**Verify it works:**

```bash
# From another computer on same network:
ping starnav-fc.local
ssh pi@starnav-fc.local
```

### Install Development Tools

```bash
# Essential build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-venv \
    htop \
    vim

# Serial communication tools
sudo apt install -y \
    minicom \
    screen \
    cu

# Development libraries
sudo apt install -y \
    libyaml-dev \
    libssl-dev \
    libffi-dev \
    python3-dev \
    i2c-tools
```

---

## UART Configuration

### Enable UART Hardware

**Edit boot configuration:**

```bash
sudo nano /boot/firmware/config.txt
```

**Add at end of file:**

```ini
# Enable UART for STM32 communication
enable_uart=1

# Disable Bluetooth to free up UART0
dtoverlay=disable-bt
```

**Save and exit** (Ctrl+X, Y, Enter)

### Disable Serial Console

```bash
sudo raspi-config
```

**Navigate:**

```
3 Interface Options
  → I6 Serial Port
    → "Login shell over serial?" → No
    → "Serial port hardware enabled?" → Yes
  → Finish
  → Reboot? → Yes
```

### Verify UART

```bash
# After reboot, check device exists
ls -l /dev/ttyAMA0

# Should show:
# crw-rw---- 1 root dialout ... /dev/ttyAMA0

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in, or:
newgrp dialout
```

**Test UART loopback:**

```bash
# Connect GPIO 14 to GPIO 15 physically
# Terminal 1:
cat /dev/ttyAMA0

# Terminal 2:
echo "test" > /dev/ttyAMA0

# You should see "test" in terminal 1
```

---

## GPIO Configuration (Pi 5 Specific)

### lgpio Library

> 💡 **Important**: Pi 5 uses RP1 chip for GPIO, requiring `lgpio` instead of deprecated `pigpio`.

**Install system library:**

```bash
sudo apt install -y python3-lgpio python3-gpiozero

# Verify installation
python3 -c "import lgpio; print(f'lgpio version: {lgpio.version()}')"
```

**Add user to gpio group:**

```bash
sudo usermod -a -G gpio $USER
newgrp gpio
```

### GPIO Pin Reference

| Function   | BCM GPIO | Physical Pin | Alt Function |
| ---------- | -------- | ------------ | ------------ |
| UART TX    | 14       | Pin 8        | TXD0         |
| UART RX    | 15       | Pin 10       | RXD0         |
| I2C SDA    | 2        | Pin 3        | SDA1         |
| I2C SCL    | 3        | Pin 5        | SCL1         |
| Servo Pan  | 17       | Pin 11       | GPIO         |
| Servo Tilt | 27       | Pin 13       | GPIO         |

---

## I2C Configuration

### Enable I2C

```bash
sudo raspi-config
```

**Navigate:**

```
3 Interface Options
  → I5 I2C
    → Enable? → Yes
```

**Increase I2C speed (optional):**

```bash
sudo nano /boot/firmware/config.txt
```

**Add:**

```ini
# I2C configuration
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000  # 400kHz (fast mode)
```

### Verify I2C

```bash
# Check devices exist
ls /dev/i2c*
# Should show: /dev/i2c-1

# Scan for devices (after connecting PCA9685)
i2cdetect -y 1

# Expected output (with PCA9685 at 0x40):
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# ...
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
```

---

## Performance Optimization

### Disable Unnecessary Services

```bash
# Free up resources
sudo systemctl disable bluetooth
sudo systemctl disable triggerhappy

# Check what's running
systemctl list-units --type=service --state=running
```

> ⚠️ **Note**: Keep `avahi-daemon` enabled if you want to access Pi via `.local` hostname.

### Optional Overclocking

⚠️ **Warning**: Requires active cooling. Monitor temperatures.

```bash
sudo nano /boot/firmware/config.txt
```

**Add (use cautiously):**

```ini
# Overclocking (requires cooling!)
over_voltage=4
arm_freq=2400

# Monitor with: vcgencmd measure_temp
```

### GPU Memory Split

```bash
# Reduce GPU memory for headless operation
sudo raspi-config
```

**Navigate:**

```
4 Performance Options
  → P2 GPU Memory
    → Set to: 16 (minimum for headless)
```

---

## Python Environment

### Create Virtual Environment

```bash
# Create workspace
mkdir -p ~/workspace
cd ~/workspace

# Create virtual environment
python3 -m venv starnav-env

# Activate
source starnav-env/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install required packages
pip install \
    flask \
    flask-socketio \
    python-socketio \
    gpiozero \
    lgpio \
    pyserial
```

**Create activation script:**

```bash
nano ~/activate-starnav.sh
```

**Content:**

```bash
#!/bin/bash
source ~/workspace/starnav-env/bin/activate
cd ~/workspace/stellar-navigation
echo "Stellar Navigation environment activated"
```

**Make executable:**

```bash
chmod +x ~/activate-starnav.sh
```

---

## Directory Structure

```bash
mkdir -p ~/workspace/stellar-navigation/{docs,scripts,dashboard,logs}
```

**Recommended layout:**

```
~/workspace/
├── cFS/                    # NASA Core Flight System
│   ├── apps/
│   │   └── starnav/        # Custom STARNAV app
│   ├── build/
│   └── cfe/
├── stellar-navigation/      # Project repository
│   ├── dashboard/          # Web interface
│   ├── scripts/            # Utility scripts
│   ├── docs/               # Documentation
│   └── logs/               # Telemetry logs
└── starnav-env/            # Python virtual environment
```

---

## Auto-Start Configuration

### Create systemd Service

**cFS service:**

```bash
sudo nano /etc/systemd/system/cfs-starnav.service
```

**Content:**

```ini
[Unit]
Description=Core Flight System - Stellar Navigation
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/workspace/cFS/build/exe/cpu1
ExecStart=/home/pi/workspace/cFS/build/exe/cpu1/core-cpu1
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Python bridge service:**

```bash
sudo nano /etc/systemd/system/starnav-bridge.service
```

**Content:**

```ini
[Unit]
Description=Stellar Navigation Python Bridge
After=cfs-starnav.service
Requires=cfs-starnav.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/workspace/stellar-navigation/dashboard
Environment="PATH=/home/pi/workspace/starnav-env/bin:/usr/local/bin:/usr/bin"
ExecStart=/home/pi/workspace/starnav-env/bin/python3 cfs_bridge.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Enable services:**

```bash
# Enable but don't start yet
sudo systemctl enable cfs-starnav.service
sudo systemctl enable starnav-bridge.service

# Start manually when ready:
# sudo systemctl start cfs-starnav.service
# sudo systemctl start starnav-bridge.service

# Check status:
# sudo systemctl status cfs-starnav.service
```

---

## Network Configuration

### Static IP (Optional)

**For Ethernet:**

```bash
sudo nano /etc/dhcpcd.conf
```

**Add:**

```
interface eth0
static ip_address=192.168.1.100/24
static routers=192.168.1.1
static domain_name_servers=192.168.1.1 8.8.8.8
```

### Firewall

```bash
# Install (optional)
sudo apt install ufw

# Allow SSH
sudo ufw allow 22/tcp

# Allow web dashboard
sudo ufw allow 5000/tcp

# Enable
sudo ufw enable
```

---

## Monitoring & Maintenance

### Temperature Monitoring

```bash
# Current temperature
vcgencmd measure_temp

# Continuous monitoring
watch -n 1 vcgencmd measure_temp

# Target: <70°C normal, <80°C under load
```

### Resource Usage

```bash
# Real-time monitoring
htop

# Disk usage
df -h

# Memory usage
free -h

# Process list
ps aux | grep -E 'cfs|python'
```

### Log Management

```bash
# View cFS logs
journalctl -u cfs-starnav.service -f

# View Python bridge logs
journalctl -u starnav-bridge.service -f

# Clear old logs
sudo journalctl --vacuum-time=7d
```

---

## Backup & Recovery

### Create SD Card Backup

**From Linux:**

```bash
# Identify SD card
lsblk

# Create image (replace sdX with actual device)
sudo dd if=/dev/sdX of=starnav-backup.img bs=4M status=progress

# Compress
gzip starnav-backup.img
```

**From Windows:**

- Use Win32DiskImager
- Or: Raspberry Pi Imager (Read mode)

### Critical Files to Backup

```bash
~/workspace/cFS/apps/starnav/
~/workspace/stellar-navigation/
~/.config/
/boot/firmware/config.txt
/etc/systemd/system/cfs-starnav.service
/etc/systemd/system/starnav-bridge.service
```

---

## Troubleshooting

### Issue: UART permission denied

**Solutions:**

```bash
# Check group membership
groups

# Should show 'dialout' - if not:
sudo usermod -a -G dialout $USER

# Log out and back in
```

### Issue: GPIO not working

**Solutions:**

```bash
# Verify lgpio installed
python3 -c "import lgpio"

# Check gpio group
groups | grep gpio

# Install if missing
sudo apt install python3-lgpio
```

### Issue: High temperature

**Solutions:**

- Add active cooling (fan)
- Reduce overclock settings
- Verify heatsink properly installed
- Check ambient temperature

### Issue: SD card corruption

**Prevention:**

- Use quality SD card (A2 rated)
- Proper shutdown (avoid power loss)
- Regular backups
- Enable read-only root (advanced)

---

## Performance Targets

| Metric          | Target  | Typical  |
| --------------- | ------- | -------- |
| CPU utilization | < 60%   | ~40%     |
| Memory usage    | < 3GB   | ~2GB     |
| Temperature     | < 70°C  | ~60°C    |
| SD write rate   | < 1MB/s | ~500KB/s |
| Network latency | < 10ms  | ~2ms     |

---

## Next Steps

After Pi 5 is configured:

1. → See `cfs-integration.md` for cFS installation
2. → See `hardware-integration.md` for wiring to STM32
3. → See `web-dashboard.md` for UI development
