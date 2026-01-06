# MPU9250 Magnetometer (AK8963) Driver - Configuration & Improvements

**Version**: 2.0  
**Date**: January 6, 2026  
**Author**: Relabs Tech / Inertial Computer Project  
**Status**: ⚠️ SPECIFICATION (Review Required Before Implementation)

---

## Executive Summary

This document specifies comprehensive improvements to the MPU9250 magnetometer (AK8963) driver, addressing **critical timing bugs** discovered through comparison with proven Arduino reference implementations. The current Go driver uses incorrect delays (2-10ms instead of required 50ms) causing unreliable magnetometer initialization and operation.

**Key Changes:**
- Fix critical timing bugs (9 out of 10 mag operations have wrong delays)
- Eliminate all hardcoded values via configuration system
- Add parameter validation and error handling
- Implement backward-compatible API with default wrappers
- Support register debug tooling with experimental timing overrides

---

## Table of Contents

1. [Problem Statement](#problem-statement)
2. [Architecture Overview](#architecture-overview)
3. [Configuration System](#configuration-system)
4. [API Changes](#api-changes)
5. [Timing Requirements](#timing-requirements)
6. [Validation Rules](#validation-rules)
7. [Implementation Plan](#implementation-plan)
8. [Testing Strategy](#testing-strategy)
9. [Arduino Reference Comparison](#arduino-reference-comparison)
10. [Migration Guide](#migration-guide)

---

## 1. Problem Statement

### Current Issues

**Critical Timing Bugs** (HIGH PRIORITY):
- `writeMagReg()` uses 2ms delay; requires 50ms per AK8963 datasheet
- `InitMag()` uses 10ms delays (5 locations); requires 50ms each
- `ConfigureMag()` uses 10ms delays; requires 50ms
- Missing magnetometer reset sequence entirely

**Architectural Issues**:
- Hardcoded resolution (16-bit), mode (continuous 100Hz), delays
- No configuration file integration
- Cannot override parameters for debugging/experimentation
- No validation of parameter ranges

**Impact**:
- Unreliable magnetometer initialization (intermittent failures)
- Incorrect ASA (factory sensitivity adjustment) reads
- Mode switching failures
- Stuck magnetometer states requiring power cycle

### Evidence

Arduino reference code (`MPU9250.cpp::initAK8963Slave`) consistently uses:
```cpp
delay(50);  // After EVERY magnetometer write operation
```

Go driver currently uses:
```go
time.Sleep(2 * time.Millisecond)   // writeMagReg() - WRONG
time.Sleep(10 * time.Millisecond)  // InitMag() x5 - WRONG
```

---

## 2. Architecture Overview

### Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    inertial_config.txt                          │
│  MAG_WRITE_DELAY_MS=50                                          │
│  MAG_READ_DELAY_MS=2                                            │
│  MAG_SCALE=1  (16-bit resolution)                               │
│  MAG_MODE=0x06  (100Hz continuous)                              │
│  REGISTER_DEBUG_MAG_WRITE_DELAY=10  (experimental override)     │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│              internal/config/config.go                          │
│  type Config struct {                                           │
│      MagWriteDelayMS int                                        │
│      MagReadDelayMS int                                         │
│      MagScale byte                                              │
│      MagMode byte                                               │
│      RegisterDebugMagWriteDelay int                             │
│  }                                                              │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│           internal/sensors/imu.go (IMUManager)                  │
│  func (m *IMUManager) Init() {                                  │
│      cfg := config.Get()                                        │
│      writeDelay := time.Duration(cfg.MagWriteDelayMS) * time.Ms │
│      readDelay := time.Duration(cfg.MagReadDelayMS) * time.Ms   │
│      src.InitMag(writeDelay, readDelay, cfg.MagScale, cfg.Mode) │
│  }                                                              │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│          internal/sensors/imu_source.go (wrapper)               │
│  func (s *imuSource) InitMag(writeDelay, readDelay, ...) {     │
│      cal, err := s.imu.InitMag(writeDelay, readDelay, ...)     │
│  }                                                              │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│             mpu9250/mpu9250.go (driver)                         │
│  func (m *MPU9250) InitMag(                                     │
│      writeDelay, readDelay time.Duration,                       │
│      scale, mode byte,                                          │
│  ) (*MagCal, error) {                                           │
│      // Uses passed parameters, no hardcoded values             │
│      time.Sleep(writeDelay)  // 50ms from config               │
│  }                                                              │
└─────────────────────────────────────────────────────────────────┘
```

### Backward Compatibility Layer

```go
// New: Fully configurable (for IMUManager, register_debug)
func (m *MPU9250) InitMag(writeDelay, readDelay time.Duration, 
                          scale, mode byte) (*MagCal, error)

// Legacy: Wrapper with defaults loaded from config
func (m *MPU9250) InitMagWithDefaults() (*MagCal, error) {
    // Reads from internal/config.Get()
    cfg := config.Get()
    return m.InitMag(
        time.Duration(cfg.MagWriteDelayMS) * time.Millisecond,
        time.Duration(cfg.MagReadDelayMS) * time.Millisecond,
        cfg.MagScale,
        cfg.MagMode,
    )
}
```

---

## 3. Configuration System

### 3.1 Standard Operation Configuration

Add to `inertial_config.txt`:

```ini
# ============================================================================
# Magnetometer (AK8963) Configuration
# ============================================================================

# Write Delay (milliseconds)
# AK8963 requires 50ms settling time after write operations per datasheet
# DO NOT change unless experimenting - 50ms is hardware requirement
MAG_WRITE_DELAY_MS=50

# Read Delay (milliseconds)  
# I2C master transaction completion time
# 2ms is sufficient for read operations
MAG_READ_DELAY_MS=2

# Magnetometer Resolution (0=14-bit, 1=16-bit)
# 0: 14-bit (0.6 µT/LSB sensitivity)
# 1: 16-bit (0.15 µT/LSB sensitivity) - RECOMMENDED
MAG_SCALE=1

# Magnetometer Operating Mode
# 0x00: Power-down mode
# 0x01: Single measurement mode
# 0x02: Continuous measurement mode 1 (8 Hz)
# 0x06: Continuous measurement mode 2 (100 Hz) - RECOMMENDED
# 0x04: External trigger measurement mode
# 0x08: Self-test mode
# 0x0F: Fuse ROM access mode (for calibration read only)
MAG_MODE=0x06

# Magnetometer Sample Rate Divider (for I2C master reads)
# Controls how often MPU9250 reads from AK8963
# 0: Read every accel/gyro sample
# 1: Read every 2nd sample (recommended to reduce I2C traffic)
# 2-15: Read every (N+1)th sample
MAG_SAMPLE_RATE_DIVIDER=1
```

### 3.2 Register Debug Tool Configuration

Add experimental timing overrides:

```ini
# ============================================================================
# Register Debug Tool - Experimental Magnetometer Timing
# ============================================================================

# Override write delay for register debugging/experimentation
# WARNING: Values < 50ms may cause unreliable magnetometer operation
# Use only for testing different timing values
# Set to -1 to use standard MAG_WRITE_DELAY_MS
REGISTER_DEBUG_MAG_WRITE_DELAY=-1

# Override read delay for register debugging
# Set to -1 to use standard MAG_READ_DELAY_MS  
REGISTER_DEBUG_MAG_READ_DELAY=-1

# Allow unsafe magnetometer operations in register debug mode
# Enables writes to reserved registers, shorter delays, etc.
# NEVER enable in production/flight systems
REGISTER_DEBUG_MAG_UNSAFE_MODE=false
```

### 3.3 Config Struct Definition

Update `internal/config/config.go`:

```go
type Config struct {
    // ... existing fields ...

    // Magnetometer Configuration
    MagWriteDelayMS          int  `config:"MAG_WRITE_DELAY_MS"`
    MagReadDelayMS           int  `config:"MAG_READ_DELAY_MS"`
    MagScale                 byte `config:"MAG_SCALE"`
    MagMode                  byte `config:"MAG_MODE"`
    MagSampleRateDivider     byte `config:"MAG_SAMPLE_RATE_DIVIDER"`
    
    // Register Debug Overrides
    RegisterDebugMagWriteDelay int  `config:"REGISTER_DEBUG_MAG_WRITE_DELAY"`
    RegisterDebugMagReadDelay  int  `config:"REGISTER_DEBUG_MAG_READ_DELAY"`
    RegisterDebugMagUnsafeMode bool `config:"REGISTER_DEBUG_MAG_UNSAFE_MODE"`
}
```

---

## 4. API Changes

### 4.1 Driver-Level Functions (mpu9250.go)

#### Current (Before)
```go
func (m *MPU9250) InitMag() (*MagCal, error)
func (m *MPU9250) ConfigureMag() error
func (m *MPU9250) ReadMagRegister(address byte) (byte, error)
func (m *MPU9250) writeMagReg(address byte, value byte) error
```

#### New (After)
```go
// Primary configurable API
func (m *MPU9250) InitMag(
    writeDelay, readDelay time.Duration,
    scale, mode byte,
) (*MagCal, error)

func (m *MPU9250) ConfigureMag(
    scale, mode byte,
    writeDelay time.Duration,
) error

func (m *MPU9250) ReadMagRegister(
    address byte,
    readDelay time.Duration,
) (byte, error)

func (m *MPU9250) writeMagReg(
    address, value byte,
    writeDelay time.Duration,
) error

// Backward-compatible convenience wrappers (load config internally)
func (m *MPU9250) InitMagWithDefaults() (*MagCal, error)
func (m *MPU9250) ConfigureMagWithDefaults() error
func (m *MPU9250) ReadMagRegisterDefault(address byte) (byte, error)
```

### 4.2 Application-Level Functions (imu_source.go)

```go
// imuSource wrapper forwards parameters from IMUManager
type IMURawReader interface {
    ReadRaw() (imu_raw.IMURaw, error)
    InitMag(writeDelay, readDelay time.Duration, scale, mode byte) (*MagCal, error)
    ReadMag(cal *MagCal) (MagData, error)
}

// Implementation
func (s *imuSource) InitMag(
    writeDelay, readDelay time.Duration,
    scale, mode byte,
) (*MagCal, error) {
    return s.imu.InitMag(writeDelay, readDelay, scale, mode)
}
```

### 4.3 IMUManager Integration (imu.go)

```go
func (m *IMUManager) Init() error {
    cfg := config.Get()
    
    // Convert config values to driver parameters
    writeDelay := time.Duration(cfg.MagWriteDelayMS) * time.Millisecond
    readDelay := time.Duration(cfg.MagReadDelayMS) * time.Millisecond
    
    // Initialize left IMU magnetometer
    if m.leftIMU != nil {
        magCal, err := m.leftIMU.InitMag(
            writeDelay,
            readDelay, 
            cfg.MagScale,
            cfg.MagMode,
        )
        if err != nil {
            return fmt.Errorf("left mag init: %w", err)
        }
        m.leftMagCal = magCal
    }
    
    // Initialize right IMU magnetometer
    if m.rightIMU != nil {
        magCal, err := m.rightIMU.InitMag(
            writeDelay,
            readDelay,
            cfg.MagScale,
            cfg.MagMode,
        )
        if err != nil {
            return fmt.Errorf("right mag init: %w", err)
        }
        m.rightMagCal = magCal
    }
    
    return nil
}
```

### 4.4 Register Debug Tool Integration

```go
// cmd/register_debug uses experimental overrides
cfg := config.Get()

// Check for experimental timing override
writeDelay := time.Duration(cfg.MagWriteDelayMS) * time.Millisecond
if cfg.RegisterDebugMagWriteDelay > 0 {
    writeDelay = time.Duration(cfg.RegisterDebugMagWriteDelay) * time.Millisecond
    log.Printf("WARNING: Using experimental mag write delay: %dms", 
               cfg.RegisterDebugMagWriteDelay)
}

// Initialize with override parameters
magCal, err := imu.InitMag(writeDelay, readDelay, scale, mode)
```

---

## 5. Timing Requirements

### 5.1 AK8963 Hardware Requirements

Source: AK8963 Datasheet + Arduino Reference Implementation

| Operation | Required Delay | Current (Bug) | Status |
|-----------|---------------|---------------|--------|
| **Write to CNTL register** | 50ms | 2ms | 🔴 CRITICAL |
| **Write to CNTL2 register** | 50ms | MISSING | 🔴 CRITICAL |
| **Mode transition** | 50ms | 10ms | 🔴 HIGH |
| **Fuse ROM access** | 50ms | 10ms | 🔴 HIGH |
| **Power state change** | 50ms | 10ms | 🔴 HIGH |
| **Read single register** | 2ms | 2ms | ✅ OK |
| **Read multi-byte data** | 2ms | 2ms | ✅ OK |

### 5.2 Arduino Reference Timing

From `MPU9250.cpp::initAK8963Slave` (lines 307-355):

```cpp
// Reset magnetometer
writeByte(I2C_SLV0_DO, 0x01);
writeByte(I2C_SLV0_CTRL, 0x81);
delay(50);  // ← 50ms after write

// Power down
writeByte(I2C_SLV0_DO, 0x00);
writeByte(I2C_SLV0_CTRL, 0x81);
delay(50);  // ← 50ms after write

// Enter fuse mode
writeByte(I2C_SLV0_DO, 0x0F);
writeByte(I2C_SLV0_CTRL, 0x81);
delay(50);  // ← 50ms after write

// Read ASA values
writeByte(I2C_SLV0_CTRL, 0x83);
delay(50);  // ← 50ms for read setup

// Power down again
writeByte(I2C_SLV0_DO, 0x00);
writeByte(I2C_SLV0_CTRL, 0x81);
delay(50);  // ← 50ms after write

// Set measurement mode
writeByte(I2C_SLV0_DO, Mscale << 4 | Mmode);
writeByte(I2C_SLV0_CTRL, 0x81);
delay(50);  // ← 50ms after write
```

**Pattern**: Consistent 50ms delay after EVERY magnetometer write operation.

### 5.3 Implementation: Reset Sequence

**MISSING in current Go code** — must be added:

```go
// Arduino reference (line 288-295):
// writeByte(I2C_SLV0_REG, AK8963_CNTL2);
// writeByte(I2C_SLV0_DO, 0x01);  // Reset
// writeByte(I2C_SLV0_CTRL, 0x81);
// delay(50);

// Go implementation (add to InitMag() before line 515):
if err := m.writeMagReg(reg.MPU9250_MAG_CNTL2, 0x01, writeDelay); err != nil {
    return nil, fmt.Errorf("mag reset: %w", err)
}
```

---

## 6. Validation Rules

### 6.1 Configuration Value Validation

Implement in `internal/config/config.go::parseConfig()`:

```go
func validateMagConfig(cfg *Config) error {
    // Write delay validation
    if cfg.MagWriteDelayMS < 1 || cfg.MagWriteDelayMS > 200 {
        return fmt.Errorf("MAG_WRITE_DELAY_MS must be 1-200ms, got %d", 
                          cfg.MagWriteDelayMS)
    }
    if cfg.MagWriteDelayMS < 50 {
        log.Printf("WARNING: MAG_WRITE_DELAY_MS=%dms is below recommended 50ms",
                   cfg.MagWriteDelayMS)
    }
    
    // Read delay validation
    if cfg.MagReadDelayMS < 1 || cfg.MagReadDelayMS > 50 {
        return fmt.Errorf("MAG_READ_DELAY_MS must be 1-50ms, got %d",
                          cfg.MagReadDelayMS)
    }
    
    // Scale validation (0=14-bit, 1=16-bit)
    if cfg.MagScale > 1 {
        return fmt.Errorf("MAG_SCALE must be 0 or 1, got %d", cfg.MagScale)
    }
    
    // Mode validation (0x00-0x0F valid, but only specific values work)
    validModes := map[byte]string{
        0x00: "Power-down",
        0x01: "Single measurement",
        0x02: "Continuous 8Hz",
        0x06: "Continuous 100Hz",
        0x04: "External trigger",
        0x08: "Self-test",
        0x0F: "Fuse ROM access",
    }
    if _, ok := validModes[cfg.MagMode]; !ok {
        return fmt.Errorf("MAG_MODE=0x%02X invalid, must be one of: %v",
                          cfg.MagMode, validModes)
    }
    
    // Sample rate divider validation
    if cfg.MagSampleRateDivider > 15 {
        return fmt.Errorf("MAG_SAMPLE_RATE_DIVIDER must be 0-15, got %d",
                          cfg.MagSampleRateDivider)
    }
    
    return nil
}
```

### 6.2 Register Debug Safety Checks

```go
func validateRegisterDebugMagConfig(cfg *Config) error {
    // Check unsafe mode warnings
    if cfg.RegisterDebugMagUnsafeMode {
        log.Println("⚠️  REGISTER_DEBUG_MAG_UNSAFE_MODE=true")
        log.Println("⚠️  Unsafe magnetometer operations enabled!")
        log.Println("⚠️  DO NOT use in production systems")
    }
    
    // Validate override delays (if set)
    if cfg.RegisterDebugMagWriteDelay > 0 {
        if cfg.RegisterDebugMagWriteDelay < 1 || 
           cfg.RegisterDebugMagWriteDelay > 500 {
            return fmt.Errorf("REGISTER_DEBUG_MAG_WRITE_DELAY out of range: %d",
                              cfg.RegisterDebugMagWriteDelay)
        }
        if cfg.RegisterDebugMagWriteDelay < 50 && 
           !cfg.RegisterDebugMagUnsafeMode {
            return fmt.Errorf("REGISTER_DEBUG_MAG_WRITE_DELAY < 50ms requires UNSAFE_MODE=true")
        }
    }
    
    if cfg.RegisterDebugMagReadDelay > 0 {
        if cfg.RegisterDebugMagReadDelay < 1 || 
           cfg.RegisterDebugMagReadDelay > 100 {
            return fmt.Errorf("REGISTER_DEBUG_MAG_READ_DELAY out of range: %d",
                              cfg.RegisterDebugMagReadDelay)
        }
    }
    
    return nil
}
```

### 6.3 Runtime Parameter Validation

Implement in `mpu9250.go`:

```go
func validateMagParams(writeDelay, readDelay time.Duration, 
                       scale, mode byte) error {
    if writeDelay < time.Millisecond {
        return fmt.Errorf("writeDelay too short: %v", writeDelay)
    }
    if readDelay < time.Millisecond {
        return fmt.Errorf("readDelay too short: %v", readDelay)
    }
    if scale > 1 {
        return fmt.Errorf("invalid scale: %d (must be 0 or 1)", scale)
    }
    // Mode validation per AK8963 datasheet
    validModes := map[byte]bool{
        0x00: true, 0x01: true, 0x02: true, 
        0x04: true, 0x06: true, 0x08: true, 0x0F: true,
    }
    if !validModes[mode] {
        return fmt.Errorf("invalid mode: 0x%02X", mode)
    }
    return nil
}
```

---

## 7. Implementation Plan

### Phase 1: Configuration System

**Files to modify:**
- `inertial_config.txt` — Add magnetometer configuration section
- `internal/config/config.go` — Add Config struct fields, parsing, validation

**Tasks:**
1. Add configuration entries to `inertial_config.txt`
2. Update `Config` struct with magnetometer fields
3. Implement `parseConfig()` magnetometer parsing
4. Implement `validateMagConfig()` validation function
5. Add unit tests for config parsing and validation
6. Document default values and rationale

**Deliverables:**
- [ ] Config file updated with documented parameters
- [ ] Config struct supports all mag parameters
- [ ] Validation prevents invalid configurations
- [ ] Unit tests pass for all valid/invalid combinations

### Phase 2: Driver API Refactoring

**Files to modify:**
- `mpu9250/mpu9250.go` — Update all magnetometer functions

**Tasks:**
1. Update function signatures with timing/mode parameters
2. Add reset sequence to `InitMag()`
3. Fix all timing delays (50ms for writes)
4. Remove all hardcoded values
5. Add parameter validation
6. Implement backward-compatible wrapper functions
7. Update internal `writeMagReg()` signature
8. Add inline documentation with datasheet references

**Function changes:**
```go
// BEFORE
func InitMag() (*MagCal, error)

// AFTER  
func InitMag(writeDelay, readDelay time.Duration, scale, mode byte) (*MagCal, error)
func InitMagWithDefaults() (*MagCal, error)  // Wrapper
```

**Deliverables:**
- [ ] All magnetometer functions accept timing parameters
- [ ] All delays use passed parameters (no hardcoded values)
- [ ] Reset sequence implemented
- [ ] Wrapper functions for backward compatibility
- [ ] Code documentation updated

### Phase 3: IMU Manager Integration

**Files to modify:**
- `internal/sensors/imu.go` — IMUManager Init() updates
- `internal/sensors/imu_source.go` — Wrapper interface updates

**Tasks:**
1. Update `IMUManager.Init()` to load config values
2. Pass config parameters to driver functions
3. Update `imuSource` wrapper interface
4. Handle left/right IMU magnetometer initialization separately
5. Store MagCal results in IMUManager
6. Add error handling and logging

**Deliverables:**
- [ ] IMUManager loads mag config at startup
- [ ] Parameters passed correctly to both left/right IMUs
- [ ] Error handling for init failures
- [ ] Logging shows which config values are used

### Phase 4: Register Debug Tool Support

**Files to modify:**
- `cmd/register_debug/` — Add magnetometer override support
- `internal/app/register_debug.go` — Experimental timing logic

**Tasks:**
1. Add override config parsing in register debug
2. Implement timing override selection logic
3. Add safety warnings when using unsafe values
4. Enable experimental timing testing
5. Add CLI flags for runtime override (optional)

**Deliverables:**
- [ ] Register debug can use experimental timing
- [ ] Safety warnings displayed for < 50ms writes
- [ ] Override parameters logged clearly
- [ ] Standard operation unaffected

### Phase 5: Testing & Validation

**Test cases:**
1. **Unit tests** (config parsing, validation, parameter passing)
2. **Integration tests** (IMU manager initialization)
3. **Hardware tests** (real MPU9250 + AK8963):
   - Magnetometer init success rate (target: 100%)
   - ASA value consistency (should be 0.9-1.2 range)
   - Mode switching reliability
   - Timing variation experiments (register debug)
4. **Regression tests** (ensure accel/gyro unaffected)

**Deliverables:**
- [ ] All unit tests pass
- [ ] Hardware tests show reliable mag operation
- [ ] ASA values consistent across reboots
- [ ] No regression in IMU operation
- [ ] Register debug timing experiments documented

---

## 8. Testing Strategy

### 8.1 Unit Tests

```go
// internal/config/config_test.go
func TestMagConfigParsing(t *testing.T)
func TestMagConfigValidation(t *testing.T)
func TestMagConfigDefaults(t *testing.T)
func TestRegisterDebugOverrides(t *testing.T)

// mpu9250/mpu9250_test.go
func TestInitMagWithParams(t *testing.T)
func TestInitMagValidation(t *testing.T)
func TestMagTimingParameters(t *testing.T)
func TestBackwardCompatibleWrappers(t *testing.T)
```

### 8.2 Integration Tests

```go
// internal/sensors/imu_test.go
func TestIMUManagerMagInit(t *testing.T)
func TestDualIMUMagInit(t *testing.T)
func TestMagInitFailureHandling(t *testing.T)
```

### 8.3 Hardware Tests

**Test Procedure:**
1. Configure with standard timing (50ms write, 2ms read)
2. Initialize magnetometer 100 times
3. Record success rate (target: 100%)
4. Verify ASA values consistent (± 0.01 across runs)
5. Test mode switching (8Hz ↔ 100Hz)
6. Test resolution switching (14-bit ↔ 16-bit)

**Timing Experiments** (register debug):
1. Test write delays: 10ms, 20ms, 30ms, 40ms, 50ms, 100ms
2. Record init success rate for each
3. Document minimum reliable delay
4. Verify 50ms is optimal

### 8.4 Regression Tests

Ensure changes don't affect accelerometer/gyroscope:
- [ ] Accel reads unchanged
- [ ] Gyro reads unchanged
- [ ] IMU initialization time acceptable
- [ ] No I2C conflicts

---

## 9. Arduino Reference Comparison

### 9.1 Reference Implementation

**Source**: `Arduino_Reference/AK8963_as_slave/MPU9250.cpp`

**Key functions:**
- `initAK8963Slave()` (lines 307-355) — Initialization sequence
- `readMagData()` (lines 271-285) — Data reading
- `getAK8963CID()` (lines 30-42) — WHO_AM_I check

### 9.2 Timing Analysis

| Operation | Arduino | Go (Current) | Go (Fixed) |
|-----------|---------|--------------|------------|
| Reset | delay(50) | MISSING | 50ms |
| Power down | delay(50) | 10ms | 50ms |
| Fuse mode | delay(50) | 10ms | 50ms |
| Read ASA | delay(50) | 10ms | 50ms |
| Power down 2 | delay(50) | 10ms | 50ms |
| Set mode | delay(50) | 10ms | 50ms |
| Read operation | delay(2) | 2ms | 2ms ✅ |

### 9.3 Sequence Comparison

#### Arduino Initialization Sequence
```cpp
1. Reset AK8963 (CNTL2=0x01) + 50ms
2. Power down (CNTL=0x00) + 50ms
3. Enter fuse mode (CNTL=0x0F) + 50ms
4. Read ASA values (3 bytes) + 50ms
5. Power down (CNTL=0x00) + 50ms
6. Set measurement mode (CNTL=scale|mode) + 50ms
7. Configure continuous read (SLV0 setup) + 50ms
```

#### Go Current Sequence (BUGGY)
```go
1. [MISSING] No reset
2. Power down (CNTL=0x00) + 10ms ❌
3. Enter fuse mode (CNTL=0x0F) + 10ms ❌
4. Read ASA values (3 bytes) + 10ms ❌
5. Power down (CNTL=0x00) + 10ms ❌
6. Set measurement mode (CNTL=0x16) + 10ms ❌
7. Configure continuous read (SLV0 setup) + 10ms ❌
```

#### Go Fixed Sequence (TARGET)
```go
1. Reset AK8963 (CNTL2=0x01) + writeDelay (50ms from config) ✅
2. Power down (CNTL=0x00) + writeDelay (50ms) ✅
3. Enter fuse mode (CNTL=0x0F) + writeDelay (50ms) ✅
4. Read ASA values (3 bytes) + readDelay (2ms) ✅
5. Power down (CNTL=0x00) + writeDelay (50ms) ✅
6. Set measurement mode (CNTL=scale|mode) + writeDelay (50ms) ✅
7. Configure continuous read (SLV0 setup) + writeDelay (50ms) ✅
```

---

## 10. Migration Guide

### 10.1 For Application Developers

**No action required** if using IMUManager:
```go
// Existing code continues to work
mgr := sensors.GetIMUManager()
mgr.Init()  // Now loads mag params from config automatically
```

**Configuration tuning:**
```ini
# inertial_config.txt
MAG_WRITE_DELAY_MS=50    # Standard (recommended)
MAG_MODE=0x06            # 100Hz continuous
MAG_SCALE=1              # 16-bit resolution
```

### 10.2 For Driver Users (Direct mpu9250 Package)

**Breaking change** — must update code:

#### Before (won't compile after changes)
```go
dev, _ := mpu9250.New(transport)
magCal, _ := dev.InitMag()  // No parameters
```

#### After (Option 1: With explicit parameters)
```go
dev, _ := mpu9250.New(transport)
magCal, _ := dev.InitMag(
    50 * time.Millisecond,  // writeDelay
    2 * time.Millisecond,   // readDelay  
    1,                       // scale (16-bit)
    0x06,                    // mode (100Hz)
)
```

#### After (Option 2: With defaults from config)
```go
dev, _ := mpu9250.New(transport)
magCal, _ := dev.InitMagWithDefaults()  // Loads from config
```

### 10.3 For Register Debug Tool

**New capabilities** — experimental timing:

```ini
# inertial_config.txt
REGISTER_DEBUG_MAG_WRITE_DELAY=10  # Experiment with 10ms
REGISTER_DEBUG_MAG_UNSAFE_MODE=true # Enable unsafe mode
```

```go
// register_debug automatically uses overrides
cfg := config.Get()
if cfg.RegisterDebugMagWriteDelay > 0 {
    log.Printf("Using experimental write delay: %dms", 
               cfg.RegisterDebugMagWriteDelay)
}
```

---

## 11. Documentation Updates Required

### 11.1 Files to Update

- [x] `mpu9250/MAG_README.md` (this file)
- [ ] `mpu9250/mpu9250.go` — Function documentation
- [ ] `internal/config/config.go` — Config field documentation
- [ ] `inertial_config.txt` — Magnetometer section with examples
- [ ] `ARCHITECTURE.md` — Magnetometer timing section
- [ ] `TODO.md` — Remove completed magnetometer items
- [ ] `QUICKSTART.md` — Mention magnetometer configuration

### 11.2 Inline Code Documentation

Add to all updated functions:

```go
// InitMag initializes the AK8963 magnetometer via MPU9250's I2C master.
//
// Parameters:
//   writeDelay: Delay after magnetometer write operations. 
//               AK8963 datasheet requires minimum 50ms for reliable operation.
//               Arduino reference uses 50ms consistently.
//   readDelay:  Delay for I2C master read completion.
//               2ms is sufficient per datasheet.
//   scale:      Resolution. 0=14-bit (0.6µT/LSB), 1=16-bit (0.15µT/LSB).
//   mode:       Operating mode. 0x02=8Hz continuous, 0x06=100Hz continuous.
//
// Returns factory sensitivity adjustment values (MagCal) for X/Y/Z axes.
//
// Sequence (matching Arduino initAK8963Slave):
//   1. Reset AK8963 via CNTL2 register
//   2. Power down magnetometer
//   3. Enter fuse ROM access mode
//   4. Read ASA (sensitivity adjustment) values
//   5. Power down again
//   6. Set measurement mode and resolution
//   7. Configure I2C master for continuous read
//
// Reference: Arduino_Reference/AK8963_as_slave/MPU9250.cpp lines 307-355
func (m *MPU9250) InitMag(writeDelay, readDelay time.Duration, 
                          scale, mode byte) (*MagCal, error) {
    // ...
}
```

---

## 12. Open Questions & Decisions Needed

### 12.1 Configuration Defaults

**Decision**: Start with simple single-set defaults (approved)

### 12.2 Runtime Reconfiguration

**Decision**: Implement `ConfigureMag()` refactor first, defer runtime switching

### 12.3 Calibration Storage

**Decision**: Defer to separate magnetometer calibration feature

---

## 13. Success Criteria

### 13.1 Functional Requirements

- [ ] Magnetometer initialization success rate ≥ 99% (100 consecutive tests)
- [ ] ASA values consistent across reboots (± 0.01 variation)
- [ ] Mode switching works reliably (8Hz ↔ 100Hz)
- [ ] Resolution switching works (14-bit ↔ 16-bit)
- [ ] No regression in accelerometer/gyroscope operation
- [ ] Configuration validation catches invalid values
- [ ] Register debug tool can override timing parameters

### 13.2 Code Quality Requirements

- [ ] Zero hardcoded magnetometer parameters in driver
- [ ] All functions have parameter validation
- [ ] Backward-compatible wrappers exist for existing code
- [ ] Unit test coverage ≥ 80% for mag-related code
- [ ] Integration tests pass on real hardware
- [ ] Documentation complete and accurate
- [ ] Arduino reference timing matched exactly

### 13.3 Performance Requirements

- [ ] Magnetometer initialization time ≤ 500ms
- [ ] Read latency ≤ 5ms (at 100Hz mode)
- [ ] No I2C timeouts or communication errors
- [ ] Memory usage unchanged (no leaks)

---

## 14. References

### Datasheets
- [MPU-9250 Product Specification](https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [MPU-9250 Register Map](https://www.invensense.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf)
- [AK8963 Datasheet](https://www.alldatasheet.com/datasheet-pdf/pdf/535561/AKM/AK8963.html)

### Code References
- Arduino Reference: `Arduino_Reference/AK8963_as_slave/MPU9250.cpp`
- Current Go Driver: `mpu9250/mpu9250.go`
- Config System: `internal/config/config.go`
- IMU Manager: `internal/sensors/imu.go`

### Related Documentation
- [ARCHITECTURE.md](../../inertial_computer/ARCHITECTURE.md) — System architecture
- [TODO.md](../../inertial_computer/TODO.md) — Project tasks
- [CALIBRATION_UI.md](../../inertial_computer/CALIBRATION_UI.md) — Calibration guide

---

## 15. Review Checklist

**Before implementation begins**, verify:

- [ ] Configuration parameter names are clear and documented
- [ ] Validation rules are comprehensive and prevent invalid states
- [ ] API changes are backward-compatible where possible
- [ ] Breaking changes are clearly documented
- [ ] Register debug safety mechanisms are sufficient
- [ ] Testing strategy covers all critical paths
- [ ] Arduino reference comparison is accurate
- [ ] Migration guide addresses all affected code
- [ ] Open questions have decisions or are deferred appropriately

**Reviewers**: Please comment on:
1. Configuration parameter naming
2. Validation strictness (too strict? too lenient?)
3. Register debug safety (sufficient warnings?)
4. API design (parameter ordering, naming)
5. Migration impact (breaking changes acceptable?)
6. Testing coverage (missing scenarios?)

---

**Status**: ⚠️ AWAITING REVIEW — Do not proceed with implementation until approved.

**Next Steps After Approval**:
1. ✅ Created feature branch: `refactor/magnetometer_code` (both repos)
2. Implement Phase 1 (Configuration System)
3. Code review after each phase
4. Hardware testing after Phase 3
5. Merge to main after Phase 5 complete

---

**End of MAG_README.md**
