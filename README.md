# STM32H743 MPU-6050 Bring-Up

A minimal embedded systems project to bring up the onboard MPU-6050 six-axis IMU on the Wildfire STM32H743XI EVK PRO development board.

The initial goal is to verify I²C communication, validate sensor initialization, and build a timestamped sensor data pipeline on the STM32H743. The project will serve as the hardware bring-up foundation for a real-time sensor processing platform.

## 1. Project Objectives

- Verify communication with the onboard MPU-6050 over I²C.
- Read and validate the `WHO_AM_I` register.
- Initialize the sensor and configure its digital low-pass filter and sample rate.
- Read accelerometer, temperature, and gyroscope measurements.
- Capture data-ready timestamps using an STM32 hardware timer.
- Evaluate sampling frequency, timing jitter, and stationary sensor behavior.
- Establish a foundation for a future UART sensor data stream to a Raspberry Pi 5.

## 2. Hardware

| Component | Description |
|---|---|
| Development board | Wildfire STM32H743XI EVK PRO |
| MCU | STM32H743XIH6 |
| IMU | Onboard MPU-6050 six-axis accelerometer and gyroscope |
| Development host | Ubuntu 24.04 |
| Compiler | `arm-none-eabi-gcc` |
| Debugging | OpenOCD and CMSIS-DAP |

The onboard MPU-6050 avoids the need for an external IMU module during initial bring-up.

## 3. I²C Configuration

The initial configuration targets I²C1 using the following MCU pins:

| Signal | MCU Pin |
|---|---|
| I²C1 SCL | PB8 |
| I²C1 SDA | PB9 |

Initial bus speed: **100 kHz**.

The pin assignments and onboard sensor connection should be verified against the board schematics and the generated STM32CubeMX configuration before testing.

### MPU-6050 Registers

| Register | Address | Expected value or purpose |
|---|---:|---|
| `WHO_AM_I` | `0x75` | Expected identity value: `0x68` |
| `PWR_MGMT_1` | `0x6B` | Wake the sensor and select a clock source |
| `SMPLRT_DIV` | `0x19` | Configure the sample-rate divider |
| `CONFIG` | `0x1A` | Configure the digital low-pass filter |
| `INT_ENABLE` | `0x38` | Enable data-ready interrupts |

The MPU-6050 uses 7-bit I²C address `0x68` when AD0 is low. STM32 HAL I²C APIs commonly take the address shifted left by one bit, resulting in `0xD0`.

## 4. Development Plan

### Phase 1: I²C Bring-Up

- [ ] Create the STM32H743 project.
- [ ] Configure I²C1 on PB8 and PB9.
- [ ] Configure the I²C bus for 100 kHz.
- [ ] Read `WHO_AM_I` at register `0x75`.
- [ ] Confirm that the returned value is `0x68`.

### Phase 2: Sensor Initialization

- [ ] Implement sensor reset and initialization.
- [ ] Clear the sleep bit and select the clock source.
- [ ] Configure the digital low-pass filter.
- [ ] Configure the sample-rate divider.
- [ ] Verify register values after initialization.

### Phase 3: Sensor Data Acquisition

- [ ] Read the 14-byte data block beginning at register `0x3B`.
- [ ] Decode accelerometer, temperature, and gyroscope raw values.
- [ ] Validate measurements while the board is stationary.
- [ ] Record gyroscope bias and accelerometer magnitude.

### Phase 4: Timestamping and Timing Analysis

- [ ] Identify and verify the MPU-6050 interrupt GPIO.
- [ ] Configure a hardware timer for microsecond-resolution timestamps.
- [ ] Capture timestamps on data-ready events.
- [ ] Measure sampling frequency and timestamp intervals.
- [ ] Analyze timing variation and missed samples.

### Phase 5: Data Transport

- [ ] Define a packet containing a sequence number, timestamp, sensor measurements, and CRC.
- [ ] Implement packet framing using COBS.
- [ ] Transmit sensor data over UART.
- [ ] Validate packet integrity and sequence continuity.

Integration with the Raspberry Pi 5 is a later milestone and is not required for the initial sensor bring-up.

## 5. Software Environment

The development environment uses:

- Ubuntu 24.04
- GNU Arm Embedded Toolchain
- STM32 HAL
- OpenOCD
- STM32CubeMX for peripheral configuration and code generation

The exact build commands will be documented after the STM32 project structure and build system are established.

## 6. Validation and Results

Validation results will be recorded here as experiments are completed.

| Test | Expected result | Status |
|---|---|---|
| I²C communication | Successful register read | Pending |
| `WHO_AM_I` | `0x68` | Pending |
| Sensor initialization | Configuration registers verified | Pending |
| Stationary accelerometer | Magnitude approximately 1 g | Pending |
| Sampling interval | Consistent with configured sample rate | Pending |
| Timestamp jitter | Measured and reported | Pending |
| UART packet integrity | Valid CRC and continuous sequence numbers | Pending |

Measured results will be added after testing. No performance or timing results are claimed until they have been experimentally verified.

## 7. Repository Scope

This repository focuses on low-level embedded development and measurable hardware behavior:

- Peripheral configuration and register access
- Sensor driver design
- Real-time data acquisition
- Timestamping and timing analysis
- Communication protocol design
- Hardware-oriented validation

The broader real-time sensor platform may later integrate STM32 firmware with a Raspberry Pi 5 processing pipeline.

## 8. License

License: To be determined.
