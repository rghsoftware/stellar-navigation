# Stellar Navigation Development Guide

> **Living Document** - Last Updated: November 15, 2025  
> **Purpose**: Track implementation progress, decisions, and next actions  
> **Repository**: [github.com/rghsoftware/stellar-navigation](https://github.com/rghsoftware/stellar-navigation)

---

## 🚀 Quick Status Dashboard

**Current Phase**: Foundation Setup  
**Algorithm**: Triangle (starting point)  
**Hardware Status**: Components ordered/arriving  
**Next Major Milestone**: First successful star identification

### Progress Overview

- [x] System design documented
- [x] Hardware architecture defined
- [ ] STM32 development environment
- [ ] Basic UART communication
- [ ] Triangle algorithm implementation
- [ ] cFS integration
- [ ] Web dashboard
- [ ] Physical gimbal control

---

## 📋 Phase 0: Prerequisites & Setup (✅ Complete when ready)

### Hardware Checklist

- [x] STM32F407VET6 board received
- [x] ST-Link V2 programmer available
- [x] Raspberry Pi 5 setup (_reference: [Hardware Integration](docs/components/hardware-integration.md)_)
- [x] Arducam B0283 gimbal kit assembled
- [ ] Wiring harness prepared

### Software Environment

- [ ] STM32 development environment:
  - [ ] **RTOS Decision** (choose one):
    - [x] Zephyr RTOS
    - [ ] FreeRTOS
    - [ ] Bare metal (no RTOS)

  - [x] **Development Environment** (choose one):
    - [ ] STM32CubeIDE
    - [ ] PlatformIO (VS Code)
    - [x] CMake + nvim/editor
    - [ ] Raw ARM toolchain + makefiles

  - [x] **Required regardless of choice**:
    - [x] arm-none-eabi-gcc compiler
    - [x] Programming tool (dfu-util or stm32flash)
    - [x] Serial terminal for debugging

- [x] Pi development environment:
  - [x] cFS cloned and test-compiled
  - [x] Python 3.9+ with uv venv
  - [x] I2C enabled (_reference: [Raspberry Pi Setup](docs/components/raspberry-pi-setup.md)_)
  - [x] UART enabled (console disabled)

### Test Data Preparation

- [x] Star catalog selected:
  - [x] Yale Bright Star Catalog (BSC5) filtered to magnitude 6.5
  - [ ] Conversion to binary format
- [ ] Synthetic test generator ready
- [ ] 20 nominal test cases created
- [ ] Ground truth validation data

**Decision Log**:

- **RTOS Choice**: Zephyr was chosen because it has modern device tree, complete driver ecosystem, strong STM32
  support, and better maintainability.
- **Catalog Format**: [TBD - Binary structure definition]
- **Test Data Delivery**: [TBD - Compile-time vs UART streaming]

---

## 🔨 Phase 1: Hardware Integration & Setup

**Goal**: Complete hardware assembly and basic system setup  
**Timeline**: Week 1 (8-12 hours)  
**Success Criteria**: All hardware connected, Pi configured, cFS building

### Hardware Assembly

#### Milestone 1.1: Component Integration

- [ ] Complete wiring harness (_reference: [Hardware Integration](docs/components/hardware-integration.md)_)
  - [x] STM32 ↔ Pi UART connection (TX/RX crossed)
  - [x] Pi ↔ PCA9685 I2C connection (400kHz)
- [ ] Physical assembly
  - [x] Gimbal mounted and tested
  - [x] Servo calibration completed
  - [ ] Cable management secured

**Verification**: All components powered, no shorts, basic servo movement

### Raspberry Pi Configuration

#### Milestone 1.2: Pi 5 Setup

- [x] System configuration (_reference: [Raspberry Pi Setup](docs/components/raspberry-pi-setup.md)_)
  - [x] OS installed and updated
  - [x] I2C and UART enabled in raspi-config
  - [x] Console disabled on UART
  - [x] Python 3.9+ environment with uv
- [x] Development tools
  - [x] Git configured
  - [x] SSH access working
  - [x] Network connectivity verified

**Verification**: `i2cdetect -y 1` shows PCA9685, UART accessible

### cFS Framework Setup

#### Milestone 1.3: cFS Integration

- [x] cFS build environment (_reference: [cFS Integration](docs/components/cfs-integration.md)_)
  - [x] Submodules initialized
  - [x] Native compilation successful
  - [x] Mission configuration created
  - [x] STARNAV_APP template generated
- [x] Basic testing
  - [x] Core system starts
  - [x] Software bus functional
  - [x] Telemetry collection working

**Verification**: `./core-cpu1` starts without errors, basic apps running

### Development Environment

#### Milestone 1.4: Toolchain Setup

- [x] STM32 development environment
  - [x] ARM toolchain installed
  - [x] Programming tools (dfu-util/stm32flash)
  - [x] Debug terminal configured
- [ ] Dashboard environment
  - [ ] Python dependencies installed with uv
  - [ ] Flask server testable
  - [ ] WebSocket libraries ready

**Key Metrics**:

- Build time: < 2 minutes for full cFS
- Power consumption: < 15W total
- Boot time: < 30 seconds to cFS ready

---

## 🎯 Phase 2: STM32 Firmware & Algorithms

**Goal**: STM32 firmware with star identification algorithms  
**Timeline**: Week 2-3 (16-24 hours)  
**Success Criteria**: 90% identification rate on test data

### STM32 Foundation

#### Milestone 2.1: Project Bootstrap

- [ ] Create firmware structure (_reference: [STM32 Firmware](docs/components/stm32-firmware.md)_)

```text
firmware/
├── core/
│   ├── inc/
│   │   ├── main.h
│   └── src/
│       ├── main.c
├── algorithms/
├── catalog/
├── comm/
└── test/
```

- [ ] Configure system clocks (168 MHz)
- [ ] Enable FPU in compiler flags
- [ ] Setup debug UART (115200 baud)
- [ ] "Hello Stellar Nav" message working

**Verification**: `> Stellar Navigation System v0.1.0 Starting...`

#### Milestone 2.2: Star Catalog & UART

- [ ] Star catalog structure
  - [ ] Define catalog entry (ID, RA/Dec, magnitude, unit vector)
  - [ ] Implement catalog loader
  - [ ] Load 1000 stars to RAM
  - [ ] Print catalog statistics
- [ ] UART protocol implementation
  - [ ] Packet structure with CRC-16-CCITT
  - [ ] Serialization/deserialization
  - [ ] Command handler
  - [ ] Test with Python script

**Verification**: `> Loaded 1000 stars, magnitude range: 0.5-6.0`

### Algorithm Implementation

#### Milestone 2.3: Geometric Primitives

- [ ] Unit vector from pixel coordinates
- [ ] Angular distance calculation
- [ ] Quaternion operations (multiply, normalize)
- [ ] Performance profiling with FPU

**Benchmark Target**:

- Angular distance: < 1 µs
- Quaternion multiply: < 5 µs

#### Milestone 2.4: Triangle Algorithm

- [ ] Triangle database generation
  - [ ] Python script for offline generation
  - [ ] Hash table implementation
  - [ ] Tolerance handling (±0.5°)
  - [ ] Convert to C header file
- [ ] Pattern matching
  - [ ] Triangle selection from observations
  - [ ] Hash lookup implementation
  - [ ] Candidate verification (brightness, neighbors, geometry)
  - [ ] Confidence scoring
- [ ] Attitude determination
  - [ ] TRIAD implementation
  - [ ] Input: 3 matched star pairs → Output: quaternion
  - [ ] Error checking

**Database Metrics**:

- Total triangles: ~[calculate]
- Hash collisions: < 5%
- Size: < 128KB

### Testing & Integration

#### Milestone 2.5: Algorithm Validation

- [ ] Load test scenario from test-data/
- [ ] Run identification
- [ ] Compare to ground truth
- [ ] Log performance metrics
- [ ] Iterate through test suite

**Test Categories**:

- [ ] Sparse (5-8 stars): \_\_\_% success
- [ ] Nominal (10-15 stars): \_\_\_% success
- [ ] Dense (15+ stars): \_\_\_% success
- [ ] Noisy (0.1px): \_\_\_% success

**Performance Tracking**:

| Date | Test Suite | Success Rate | Avg Time | Memory |
| ---- | ---------- | ------------ | -------- | ------ |
|      | nominal_01 |              |          |        |
|      | edge_01    |              |          |        |
|      | noise_01   |              |          |        |

---

## 🌐 Phase 3: Web Dashboard & Visualization

**Goal**: Real-time web interface for system monitoring  
**Timeline**: Week 3-4 (12-16 hours)  
**Success Criteria**: Live 3D visualization, telemetry graphs

### Dashboard Foundation

#### Milestone 3.1: Web Interface

- [ ] Dashboard framework (_reference: [Web Dashboard](docs/components/web-dashboard.md)_)
  - [ ] Flask server structure
  - [ ] Three.js 3D spacecraft model
  - [ ] WebSocket real-time updates
  - [ ] Chart.js telemetry graphs
- [ ] UI Components
  - [ ] Star field visualization
  - [ ] Quaternion display (Euler angles)
  - [ ] Performance metrics panel
  - [ ] Control panel for test scenarios

**Verification**: Dashboard loads at http://localhost:5000

### Data Integration

#### Milestone 3.2: cFS Bridge

- [ ] Python middleware implementation
  - [ ] cFS Software Bus subscriber
  - [ ] Message parsing (STARNAV_ATTITUDE_MID)
  - [ ] WebSocket server
  - [ ] Error handling and reconnection
- [ ] Gimbal control interface
  - [ ] Quaternion → Euler conversion
  - [ ] Servo angle mapping
  - [ ] Smooth motion interpolation
  - [ ] Boundary limit enforcement

**Architecture Validation**:

- [ ] STM32 → UART → cFS → Python → WebSocket → Browser
- [ ] Python → I2C → PCA9685 → Servos

### User Experience

#### Milestone 3.3: Interactive Features

- [ ] Real-time updates
  - [ ] 10 Hz attitude updates
  - [ ] Live performance metrics
  - [ ] Star identification visualization
- [ ] Control capabilities
  - [ ] Test scenario selection
  - [ ] Manual gimbal control
  - [ ] System mode switching
- [ ] Data logging
  - [ ] Telemetry history
  - [ ] Performance tracking
  - [ ] Export capabilities

**Performance Targets**:

- Update rate: 10 Hz
- Latency: < 100ms
- Browser compatibility: Chrome/Firefox/Safari

---

## ⚡ Phase 4: Optimization & Advanced Features

**Goal**: Production-quality performance and robustness  
**Timeline**: Week 4-5 (16-24 hours)  
**Success Criteria**: < 100ms identification, advanced algorithms

### Performance Optimization

#### Milestone 4.1: Algorithm Profiling

- [ ] Identify hot paths with profiler
- [ ] Measure function execution times
- [ ] Memory allocation audit
- [ ] Cache usage analysis

**Bottlenecks Identified**:

1. **\*\***\_\_\_**\*\***: \_\_\_ms
2. **\*\***\_\_\_**\*\***: \_\_\_ms
3. **\*\***\_\_\_**\*\***: \_\_\_ms

#### Milestone 4.2: Optimization Techniques

- [ ] FPU optimization patterns
- [ ] Loop unrolling where beneficial
- [ ] Lookup tables for trig functions
- [ ] Memory pool for allocations
- [ ] Flash-to-RAM streaming

#### Milestone 4.3: QUEST Implementation

- [ ] Optimal attitude from N stars
- [ ] Weighted least squares
- [ ] Eigenvalue solver
- [ ] Performance comparison to TRIAD

**Accuracy Improvement**:

- TRIAD: \_\_\_° typical error
- QUEST: \_\_\_° typical error

### Advanced Algorithms

#### Milestone 4.4: Geometric Voting

- [ ] All-pairs hypothesis generation
- [ ] Attitude computation per hypothesis
- [ ] Vote accumulation matrix
- [ ] Winner selection logic

**Comparison to Triangle**:
| Metric | Triangle | Geometric Voting |
|--------|----------|------------------|
| Dense fields | | |
| Computation | | |
| Memory | | |
| Robustness | | |

#### Milestone 4.5: Tracking Mode

- [ ] State propagation from previous frame
- [ ] Star position prediction
- [ ] Nearest-neighbor matching
- [ ] Incremental attitude update
- [ ] State machine (LOST → ACQUIRING → TRACKING)

**Performance Targets**:

- Lost → Tracking: < 2 seconds
- Tracking maintenance: > 99% frames
- Recovery time: < 5 seconds

### Robustness Features

#### Milestone 4.6: Error Handling

- [ ] Outlier rejection (RANSAC-style)
- [ ] False star detection
- [ ] Partial occlusion handling
- [ ] Confidence thresholds
- [ ] Graceful degradation

#### Milestone 4.7: Edge Case Testing

- [ ] Field boundary stars
- [ ] Minimum viable stars (3-4)
- [ ] High noise (0.2px)
- [ ] False detections (20%)
- [ ] Recovery scenarios

---

## 🎯 Phase 5: System Integration & Demo

**Goal**: Complete end-to-end demonstration system  
**Timeline**: Week 5-6 (20-30 hours)  
**Success Criteria**: Full demo working with all components integrated

### End-to-End Integration

#### Milestone 5.1: Complete Data Flow

- [ ] Full pipeline validation
  - [ ] STM32 star identification → UART
  - [ ] Pi cFS STARNAV_APP → Software Bus
  - [ ] Python bridge → WebSocket + I2C
  - [ ] Web dashboard → Real-time visualization
  - [ ] Gimbal control → Physical tracking
- [ ] Performance validation
  - [ ] < 1s lost-in-space time
  - [ ] 10 Hz tracking updates
  - [ ] > 95% success rate
  - [ ] < 0.1° attitude accuracy

#### Milestone 5.2: System Testing

- [ ] Integration test suite
  - [ ] Hardware-in-the-loop testing
  - [ ] Long-duration stability testing
  - [ ] Error recovery testing
  - [ ] Performance under load
- [ ] User acceptance testing
  - [ ] Demo scenario walkthrough
  - [ ] Feature completeness validation
  - [ ] Documentation accuracy check

### Demonstration Scenarios

#### Milestone 5.3: Demo Scripts

- [ ] Basic star identification demo
  - [ ] Load test scenario
  - [ ] Show algorithm working
  - [ ] Display attitude output
- [ ] Tracking mode demo
  - [ ] Continuous attitude updates
  - [ ] Gimbal following spacecraft
  - [ ] Real-time telemetry
- [ ] Edge case demonstrations
  - [ ] Minimal star scenarios
  - [ ] Noise tolerance
  - [ ] Recovery from failures

#### Milestone 5.4: Presentation Materials

- [ ] Demo documentation
  - [ ] Quick start guide
  - [ ] Troubleshooting reference
  - [ ] Performance benchmarks
- [ ] Video documentation
  - [ ] System overview video
  - [ ] Component setup guides
  - [ ] Demo walkthrough

**Final Validation**:

- [ ] All components working together
- [ ] Performance targets met
- [ ] Documentation complete and accurate
- [ ] Ready for demonstration

---

## 📊 Performance Tracking

### Key Metrics Dashboard

**Update this table after each major milestone:**

| Metric                 | Target  | Current   | Status |
| ---------------------- | ------- | --------- | ------ |
| **Lost-in-Space Time** | < 1s    | \_\_\_ ms | 🔴🟡🟢 |
| **Tracking Update**    | < 100ms | \_\_\_ ms | 🔴🟡🟢 |
| **Success Rate**       | > 95%   | \_\_\_%   | 🔴🟡🟢 |
| **Attitude Accuracy**  | < 0.1°  | \_\_\_°   | 🔴🟡🟢 |
| **RAM Usage**          | < 64KB  | \_\_\_ KB | 🔴🟡🟢 |
| **Flash Usage**        | < 256KB | \_\_\_ KB | 🔴🟡🟢 |
| **CPU Load**           | < 50%   | \_\_\_%   | 🔴🟡🟢 |

### Test Suite Results

**Track cumulative success across test scenarios:**

| Test Category | Total  | Passed | Failed | Success % |
| ------------- | ------ | ------ | ------ | --------- |
| Nominal       | 20     |        |        |           |
| Sparse        | 10     |        |        |           |
| Dense         | 10     |        |        |           |
| Noisy         | 10     |        |        |           |
| Edge Cases    | 10     |        |        |           |
| **Total**     | **60** |        |        |           |

---

## 🐛 Issues & Blockers

### Active Issues

1. **[DATE]** - Issue: **\*\***\_\_\_\_**\*\***
   - Impact: High/Medium/Low
   - Workaround: **\*\***\_\_\_\_**\*\***
   - Resolution: **\*\***\_\_\_\_**\*\***

### Resolved Issues

1. **[DATE]** - **\*\***\_\_\_\_**\*\***
   - Solution: **\*\***\_\_\_\_**\*\***

---

## 📝 Decision Log

### Architecture Decisions

| Date | Decision           | Rationale | Alternative Considered |
| ---- | ------------------ | --------- | ---------------------- |
|      | RTOS Choice        |           |                        |
|      | Catalog Size       |           |                        |
|      | Algorithm Priority |           |                        |
|      | Test Strategy      |           |                        |

### Algorithm Parameters

**Document key parameters for reproducibility:**

- Angular tolerance: \_\_\_°
- Magnitude tolerance: \_\_\_ mag
- Minimum stars: \_\_\_
- Confidence threshold: \_\_\_
- Noise assumptions: \_\_\_ pixels
- Update rate: \_\_\_ Hz

---

## 🎯 Next Actions

### This Week

1. [ ] ***
2. [ ] ***
3. [ ] ***

### Blocked/Waiting

- [ ] Waiting for: **\*\***\_\_\_\_**\*\***
- [ ] Blocked by: **\*\***\_\_\_\_**\*\***

### Future Enhancements

- [ ] Machine learning centroiding
- [ ] Pyramid algorithm
- [ ] Cosmic ray rejection
- [ ] Multi-sensor fusion
- [ ] Hardware camera integration

---

## 📚 Quick References

### Key Documents

- [Hardware Integration](./docs/components/hardware-integration.md)
- [STM32 Firmware](./docs/components/stm32-firmware.md)
- [Raspberry Pi Setup](./docs/components/raspberry-pi-setup.md)
- [cFS Integration](./docs/components/cfs-integration.md)
- [Web Dashboard](./docs/components/web-dashboard.md)
- [Stellar Nav Quick Ref](./docs/stellar_nav_quick_ref.md)

### Critical Code Locations

- Triangle algorithm: `src/algorithms/triangle.c`
- UART protocol: `src/comm/uart_protocol.c`
- Test harness: `tools/test_runner.py`
- cFS app: `apps/starnav/`

### Performance Baselines

- Tetra: 0.014ms identification
- LOST: 95% success rate
- Commercial trackers: 1-10 arcsec

### Debugging Commands

```bash
# Monitor UART (Pi)
screen /dev/ttyAMA0 115200

# Check I2C devices
i2cdetect -y 1

# cFS console
./core-cpu1
```

---

## 📈 Progress Visualization

```
Phase 1: Hardware & Setup     [####____] 50%
Phase 2: STM32 & Algorithms    [________] 0%
Phase 3: Web Dashboard         [________] 0%
Phase 4: Optimization          [________] 0%
Phase 5: System Integration    [________] 0%

Overall Progress: ████░░░░░░░░░░░░ 20%
```

---

**Notes Section** (Lessons learned, observations, ideas):

---

---

---
