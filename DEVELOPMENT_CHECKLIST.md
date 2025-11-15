# Stellar Navigation Development Guide

> **Living Document** - Last Updated: November 2, 2025  
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
- [ ] STM32 deelopment environment
- [ ] Basic UART communication
- [ ] Triangle algorithm implementation
- [ ] cFS integration
- [ ] Web dashboard
- [ ] Physical gimbal control

---

## 📋 Phase 0: Prerequisites & Setup (✅ Complete when ready)

### Hardware Checklist

- [x] STM32F407VET6 board received
- [-] ST-Link V2 programmer available
- [ ] Raspberry Pi 5 setup (_reference: pi5_setup_guide_)
- [x] Arducam B0283 gimbal kit assembled
- [ ] 6V power supply tested (3-5A)
- [ ] Wiring harness prepared
- [ ] 1000µF capacitor installed

### Software Environment

- [ ] STM32 development environment:
  - [ ] **RTOS Decision** (choose one):
    - [ ] Zephyr RTOS
    - [ ] FreeRTOS
    - [ ] Bare metal (no RTOS)

  - [ ] **Development Environment** (choose one):
    - [ ] STM32CubeIDE
    - [ ] PlatformIO (VS Code)
    - [x] CMake + nvim/editor
    - [ ] Raw ARM toolchain + makefiles

  - [ ] **Required regardless of choice**:
    - [x] arm-none-eabi-gcc compiler
    - [x] Programming tool (dfu-util or stm32flash)
    - [x] Serial terminal for debugging

- [ ] Pi development environment:
  - [ ] cFS cloned and test-compiled
  - [x] Python 3.9+ with venv
  - [x] I2C enabled (_raspi-config_)
  - [x] UART enabled (console disabled)

### Test Data Preparation

- [ ] Star catalog selected:
  - [ ] Hipparcos subset (1000 stars for dev)
  - [ ] Conversion to binary format
- [ ] Synthetic test generator ready
- [ ] 20 nominal test cases created
- [ ] Ground truth validation data

**Decision Log**:

- **RTOS Choice**: [TBD - Zephyr vs FreeRTOS]
- **Catalog Format**: [TBD - Binary structure definition]
- **Test Data Delivery**: [TBD - Compile-time vs UART streaming]

---

## 🔨 Phase 1: Foundation Infrastructure

**Goal**: STM32 ↔ Pi communication with basic data structures  
**Timeline**: Week 1 (8-12 hours)  
**Success Criteria**: Board prints catalog stats, receives test observations

### STM32 Core Setup

#### Milestone 1.1: Project Bootstrap

- [ ] Create project structure:
  ```
  stellar-nav-stm32/
  ├── src/
  │   ├── main.c
  │   ├── catalog/
  │   ├── algorithms/
  │   └── comm/
  ├── include/
  └── test_data/
  ```
- [ ] Configure system clocks (168 MHz)
- [ ] Enable FPU in compiler flags
- [ ] Setup debug UART (115200 baud)
- [ ] "Hello Stellar Nav" message working

**Verification**: `> Stellar Navigation System v0.1.0 Starting...`

#### Milestone 1.2: Star Catalog Structure

- [ ] Define catalog entry structure:
  - Star ID (uint16_t)
  - RA/Dec (float × 2)
  - Magnitude (float)
  - Unit vector (float × 3)
- [ ] Implement catalog loader
- [ ] Load 1000 stars to RAM
- [ ] Print catalog statistics

**Verification**: `> Loaded 1000 stars, magnitude range: 0.5-6.0`

#### Milestone 1.3: UART Protocol

- [ ] Implement packet structure (_ref: Stellar_Navigation_Demo_System.md#uart-binary-protocol_)
- [ ] CRC-16-CCITT functions
- [ ] Packet serialization/deserialization
- [ ] Basic command handler
- [ ] Test with Python script

**Test Script Location**: `tools/uart_test.py`

### Pi-Side Foundation

#### Milestone 1.4: Python Test Harness

- [ ] Serial communication class
- [ ] Packet parser matching STM32 format
- [ ] Test data loader (JSON → binary)
- [ ] Results validator
- [ ] Logging framework

#### Milestone 1.5: Initial Integration Test

- [ ] STM32 receives observation from Pi
- [ ] STM32 echoes data back
- [ ] CRC validation passes
- [ ] 10 Hz update rate achieved

**Key Metrics**:

- Packet loss rate: < 0.1%
- Round-trip time: < 10ms
- Memory usage: < 32KB RAM

---

## 🎯 Phase 2: Triangle Algorithm Implementation

**Goal**: Core star identification working end-to-end  
**Timeline**: Week 2-3 (16-24 hours)  
**Success Criteria**: 90% identification rate on test data

### Algorithm Components

#### Milestone 2.1: Geometric Primitives

- [ ] Unit vector from pixel coordinates
- [ ] Angular distance calculation
- [ ] Quaternion operations (multiply, normalize)
- [ ] Performance profiling with FPU

**Benchmark Target**:

- Angular distance: < 1 µs
- Quaternion multiply: < 5 µs

#### Milestone 2.2: Triangle Database Generation

- [ ] Python script for offline generation
- [ ] Hash table implementation
- [ ] Tolerance handling (±0.5°)
- [ ] Database statistics tool
- [ ] Convert to C header file

**Database Metrics**:

- Total triangles: ~[calculate]
- Hash collisions: < 5%
- Size: < 128KB

#### Milestone 2.3: Pattern Matching

- [ ] Triangle selection from observations
- [ ] Hash lookup implementation
- [ ] Candidate verification:
  - [ ] Brightness matching
  - [ ] Neighbor consistency
  - [ ] Geometric validation
- [ ] Confidence scoring

#### Milestone 2.4: Attitude Determination

- [ ] TRIAD implementation
- [ ] Input: 3 matched star pairs
- [ ] Output: Rotation matrix
- [ ] Convert to quaternion
- [ ] Error checking

### Testing & Validation

#### Milestone 2.5: Algorithm Validation

- [ ] Load test scenario
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

## ⚡ Phase 3: Optimization & Robustness

**Goal**: Production-quality performance  
**Timeline**: Week 3-4 (12-16 hours)  
**Success Criteria**: < 100ms identification, < 64KB RAM

### Performance Optimization

#### Milestone 3.1: Profiling

- [ ] Identify hot paths with profiler
- [ ] Measure function execution times
- [ ] Memory allocation audit
- [ ] Cache usage analysis

**Bottlenecks Identified**:

1. **\*\***\_\_\_**\*\***: \_\_\_ms
2. **\*\***\_\_\_**\*\***: \_\_\_ms
3. **\*\***\_\_\_**\*\***: \_\_\_ms

#### Milestone 3.2: Algorithm Optimization

- [ ] FPU optimization patterns
- [ ] Loop unrolling where beneficial
- [ ] Lookup tables for trig functions
- [ ] Memory pool for allocations
- [ ] Flash-to-RAM streaming

#### Milestone 3.3: QUEST Implementation

- [ ] Optimal attitude from N stars
- [ ] Weighted least squares
- [ ] Eigenvalue solver
- [ ] Performance comparison to TRIAD

**Accuracy Improvement**:

- TRIAD: \_\_\_° typical error
- QUEST: \_\_\_° typical error

### Robustness Features

#### Milestone 3.4: Error Handling

- [ ] Outlier rejection (RANSAC-style)
- [ ] False star detection
- [ ] Partial occlusion handling
- [ ] Confidence thresholds
- [ ] Graceful degradation

#### Milestone 3.5: Edge Case Testing

- [ ] Field boundary stars
- [ ] Minimum viable stars (3-4)
- [ ] High noise (0.2px)
- [ ] False detections (20%)
- [ ] Recovery scenarios

---

## 🔄 Phase 4: Advanced Algorithms

**Goal**: Dual-mode operation with tracking  
**Timeline**: Week 4-5 (16-24 hours)  
**Success Criteria**: Lost-in-space < 2s, tracking < 50ms

### Geometric Voting Algorithm

#### Milestone 4.1: Implementation

- [ ] All-pairs hypothesis generation
- [ ] Attitude computation per hypothesis
- [ ] Vote accumulation matrix
- [ ] Winner selection logic
- [ ] Performance profiling

**Comparison to Triangle**:
| Metric | Triangle | Geometric Voting |
|--------|----------|------------------|
| Dense fields | | |
| Computation | | |
| Memory | | |
| Robustness | | |

### Tracking Mode

#### Milestone 4.2: Tracking Implementation

- [ ] State propagation from previous frame
- [ ] Star position prediction
- [ ] Nearest-neighbor matching
- [ ] Incremental attitude update
- [ ] Outlier rejection

#### Milestone 4.3: Mode Management

- [ ] State machine implementation:
  ```
  LOST → ACQUIRING → TRACKING
         ↑              ↓
         ← ← ← ← ← ← ← ←
  ```
- [ ] Transition criteria
- [ ] Failure recovery
- [ ] Mode status reporting

**Performance Targets**:

- Lost → Tracking: < 2 seconds
- Tracking maintenance: > 99% frames
- Recovery time: < 5 seconds

---

## 🌐 Phase 5: System Integration

**Goal**: Complete demo system operational  
**Timeline**: Week 5-6 (20-30 hours)  
**Success Criteria**: End-to-end demonstration working

### cFS Integration

#### Milestone 5.1: STARNAV_APP

- [ ] App structure from template
- [ ] UART device driver
- [ ] Software Bus messages:
  - [ ] STARNAV_ATTITUDE_MID
  - [ ] STARNAV_STATUS_MID
  - [ ] STARNAV_CMD_MID
- [ ] Message routing
- [ ] Telemetry tables

**Reference**: `cfe/cmake/sample_defs/`

#### Milestone 5.2: Python Bridge

- [ ] cFS message subscriber
- [ ] WebSocket server
- [ ] Gimbal control interface
- [ ] Data flow orchestration
- [ ] Error handling

**Architecture Validation**:

- [ ] STM32 → UART → cFS
- [ ] cFS → Python → WebSocket
- [ ] Python → I2C → Gimbal

### User Interface

#### Milestone 5.3: Web Dashboard

- [ ] Three.js 3D spacecraft model
- [ ] Real-time quaternion display
- [ ] Telemetry graphs (Chart.js)
- [ ] Star field visualization
- [ ] Control panel:
  - [ ] Mode selection
  - [ ] Test scenario loader
  - [ ] Performance metrics
- [ ] WebSocket updates

**Dashboard Components**:

```
┌─────────────────┬──────────────┐
│  3D Spacecraft  │  Telemetry   │
├─────────────────┼──────────────┤
│  Star Field     │  Controls    │
└─────────────────┴──────────────┘
```

### Physical Demonstration

#### Milestone 5.4: Gimbal Control

- [ ] Quaternion → Euler conversion
- [ ] Servo angle mapping
- [ ] Smooth motion interpolation
- [ ] Boundary limit enforcement
- [ ] Manual override mode

**Test Sequence**:

1. Random attitude → Gimbal tracks
2. Sweep test → Full range motion
3. Tracking mode → Smooth updates
4. Edge cases → Gimbal singularities

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

- [System Design](./Stellar_Navigation_Demo_System_Updated.md)
- [Algorithm Details](./Stellar_Navigation_System_Development_Updated.md)
- [Research Paper](./Star_Track_Space.pdf)

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
Phase 1: Foundation      [####____] 50%
Phase 2: Triangle Algo   [________] 0%
Phase 3: Optimization    [________] 0%
Phase 4: Advanced        [________] 0%
Phase 5: Integration     [________] 0%

Overall Progress: ████░░░░░░░░░░░░ 20%
```

---

**Notes Section** (Lessons learned, observations, ideas):

---

---

---
