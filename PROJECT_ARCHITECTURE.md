# EV Battery Management System - Project Architecture Document

**Version:** 1.0
**Date:** 2025
**Author:** EV-BMS Development Team

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Software Architecture](#2-software-architecture)
3. [Complete File Structure](#3-complete-file-structure)
4. [Module Detailed Explanation](#4-module-detailed-explanation)
   - 4.1 [Config Layer](#41-config-layer)
   - 4.2 [MCAL Layer](#42-mcal-layer)
   - 4.3 [HAL Layer](#43-hal-layer)
   - 4.4 [Services Layer](#44-services-layer)
   - 4.5 [Application Layer](#45-application-layer)
   - 4.6 [Main Application](#46-main-application)
5. [Safety Architecture (ISO 26262 FuSa)](#5-safety-architecture-iso-26262-fusa)
6. [Data Flow](#6-data-flow)
7. [Key Design Decisions](#7-key-design-decisions)

---

## 1. Project Overview

### 1.1 System Description

This project implements a modular **Electric Vehicle Battery Management System (EV-BMS)** designed for monitoring and managing lithium-ion battery packs. The system provides real-time cell voltage and temperature monitoring, passive cell balancing, thermal management, fault diagnostics, and functional safety supervision compliant with ISO 26262 ASIL-C requirements.

### 1.2 Target Hardware

| Component | Description |
|-----------|-------------|
| **MCU** | NXP FRDM-KL25Z Development Board |
| **Processor** | ARM Cortex-M0+ @ 48MHz |
| **Memory** | 128KB Flash, 16KB SRAM |
| **Battery Controller** | NXP MC33771C (14-cell AFE) |
| **Transceiver** | NXP MC33664B (SPI to TPL bridge) |
| **Display** | 16x2 I2C LCD (HD44780 + PCF8574) |

### 1.3 Hardware Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          FRDM-KL25Z MCU                                 │
│                       (ARM Cortex-M0+ @ 48MHz)                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │  SPI0   │  │ UART0   │  │  I2C0   │  │  TPM0   │  │  GPIO   │       │
│  │ (1MHz)  │  │(115200) │  │(100kHz) │  │ (25kHz) │  │         │       │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘       │
│       │            │            │            │            │             │
└───────┼────────────┼────────────┼────────────┼────────────┼─────────────┘
        │            │            │            │            │
        ▼            ▼            ▼            ▼            ▼
   ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐
   │MC33664B │  │ OpenSDA │  │ 16x2    │  │  Fan    │  │ LEDs/   │
   │Transceiver│ │ Debug   │  │  LCD    │  │  PWM    │  │Contactors│
   └────┬────┘  └─────────┘  └─────────┘  └─────────┘  └─────────┘
        │ TPL (Transformer Physical Layer)
        ▼
   ┌─────────────────────────────────────┐
   │         MC33771C Chain              │
   │  ┌─────────┐      ┌─────────┐       │
   │  │ Slave 0 │─────►│ Slave 1 │       │
   │  │ (4 cells)│      │ (4 cells)│      │
   │  └─────────┘      └─────────┘       │
   └─────────────────────────────────────┘
```

### 1.4 Key Features

- **Cell Monitoring:** 8 cells (2 slaves x 4 cells each, expandable to 14 cells per slave)
- **Temperature Monitoring:** 4 temperature sensors (2 per slave)
- **Cell Balancing:** Passive balancing via MC33771C internal MOSFETs
- **Thermal Management:** PWM-controlled fan with hysteresis
- **Safety Supervision:** ISO 26262 ASIL-C compliant fault detection
- **Debug Interface:** UART logging at 115200 baud
- **LCD Display:** Real-time SOC, voltage, and temperature display

---

## 2. Software Architecture

### 2.1 Layered Architecture Overview

The software follows a **4-layer architecture** providing clear separation of concerns, portability, and maintainability:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         APPLICATION LAYER                               │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐       │
│  │   FuSa      │ │   Cell      │ │ Diagnostics │ │  Thermal    │       │
│  │ Supervisor  │ │ Balancing   │ │   Manager   │ │  Manager    │       │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘       │
│  ┌─────────────┐ ┌─────────────┐                                       │
│  │    Fan      │ │  Battery    │                                       │
│  │  Control    │ │ Status Mon  │                                       │
│  └─────────────┘ └─────────────┘                                       │
├─────────────────────────────────────────────────────────────────────────┤
│                          SERVICES LAYER                                 │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐                       │
│  │    BMS      │ │   Debug     │ │  Watchdog   │                       │
│  │  Database   │ │ Info Manager│ │   Manager   │                       │
│  └─────────────┘ └─────────────┘ └─────────────┘                       │
├─────────────────────────────────────────────────────────────────────────┤
│                            HAL LAYER                                    │
│  ┌─────────────────────────────┐ ┌─────────────────────────────┐       │
│  │      MC33771 Driver         │ │        LCD Interface        │       │
│  │   (Battery Cell Controller) │ │    (16x2 I2C Display)       │       │
│  └─────────────────────────────┘ └─────────────────────────────┘       │
├─────────────────────────────────────────────────────────────────────────┤
│                           MCAL LAYER                                    │
│  ┌───────┐ ┌───────┐ ┌───────┐ ┌───────┐ ┌───────┐ ┌───────┐          │
│  │ GPIO  │ │ Timer │ │  PWM  │ │  I2C  │ │  SPI  │ │ UART  │          │
│  │       │ │SysTick│ │ TPM0  │ │ I2C0  │ │ SPI0  │ │ UART0 │          │
│  └───────┘ └───────┘ └───────┘ └───────┘ └───────┘ └───────┘          │
├─────────────────────────────────────────────────────────────────────────┤
│                          CONFIG LAYER                                   │
│  ┌─────────────────────────────┐ ┌─────────────────────────────┐       │
│  │       bms_config.h          │ │       board_config.h        │       │
│  │  (System Parameters)        │ │  (Hardware Pin Mapping)     │       │
│  └─────────────────────────────┘ └─────────────────────────────┘       │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Layer Responsibilities

| Layer | Responsibility | Key Characteristics |
|-------|---------------|---------------------|
| **Config** | System parameters and hardware definitions | Static configuration, no executable code |
| **MCAL** | Direct hardware register access | MCU-specific, register-level drivers |
| **HAL** | Hardware abstraction for external devices | Device-specific protocols, MCU-independent API |
| **Services** | Shared infrastructure services | Cross-cutting concerns, data management |
| **Application** | BMS-specific business logic | High-level algorithms, safety functions |

### 2.3 Design Principles

1. **Separation of Concerns:** Each layer has distinct responsibilities
2. **Dependency Inversion:** Higher layers depend on abstractions, not concrete implementations
3. **Single Responsibility:** Each module handles one specific function
4. **Configurability:** System behavior controlled through central configuration files
5. **Safety-First:** FuSa supervisor can override any module for safety

---

## 3. Complete File Structure

```
K_DemoModularBMS/
├── Config/
│   ├── bms_config.h          # BMS system parameters (cells, limits, timing)
│   └── board_config.h        # Hardware pin assignments (FRDM-KL25Z specific)
│
├── MCAL/
│   ├── gpio.h                # GPIO driver header
│   ├── gpio.c                # GPIO driver implementation
│   ├── timer.h               # SysTick timer driver header
│   ├── timer.c               # SysTick timer implementation
│   ├── pwm.h                 # TPM/PWM driver header
│   ├── pwm.c                 # TPM/PWM driver implementation
│   ├── i2c.h                 # I2C driver header
│   ├── i2c.c                 # I2C driver implementation
│   ├── spi_mc33664b.h        # SPI driver header (for MC33664B)
│   ├── spi_mc33664b.c        # SPI driver implementation
│   ├── uart_kl25z.h          # UART driver header
│   └── uart_kl25z.c          # UART driver implementation
│
├── HAL/
│   ├── mc33771_driver.h      # MC33771C battery controller driver header
│   ├── mc33771_driver.c      # MC33771C driver implementation
│   ├── lcd_if.h              # LCD interface driver header
│   └── lcd_if.c              # LCD driver implementation
│
├── Services/
│   ├── bms_database.h        # Central data repository header
│   ├── bms_database.c        # BMS database implementation
│   ├── debug_info_mgr.h      # Debug logging service header
│   ├── debug_info_mgr.c      # Debug logging implementation
│   ├── wdg_mgr.h             # Watchdog manager header
│   └── wdg_mgr.c             # Watchdog manager implementation
│
├── Application/
│   ├── fusa_supervisor.h     # Functional safety supervisor header
│   ├── fusa_supervisor.c     # FuSa supervisor implementation
│   ├── cell_balancing_mgr.h  # Cell balancing manager header
│   ├── cell_balancing_mgr.c  # Cell balancing implementation
│   ├── diagnostics_mgr.h     # Diagnostics manager header
│   ├── diagnostics_mgr.c     # Diagnostics implementation
│   ├── thermal_mgr.h         # Thermal manager header
│   ├── thermal_mgr.c         # Thermal manager implementation
│   ├── fan_control.h         # Fan control module header
│   ├── fan_control.c         # Fan control implementation
│   ├── battery_status_mon.h  # Battery status monitor header
│   └── battery_status_mon.c  # Battery status implementation
│
├── main.c                    # Application entry point and main loop
└── PROJECT_ARCHITECTURE.md   # This document
```

### 3.1 File Count Summary

| Category | Files | Lines of Code (approx.) |
|----------|-------|------------------------|
| Config | 2 headers | ~430 |
| MCAL | 6 headers + 6 sources | ~1,200 |
| HAL | 2 headers + 2 sources | ~700 |
| Services | 3 headers + 3 sources | ~200 |
| Application | 6 headers + 6 sources | ~1,400 |
| Main | 1 source | ~320 |
| **Total** | **30 files** | **~4,250** |

---

## 4. Module Detailed Explanation

### 4.1 Config Layer

#### 4.1.1 bms_config.h

**Purpose:** Central configuration file containing all BMS system parameters.

**Design Rationale:**
- Single source of truth for all configurable parameters
- Easy system tuning without code changes
- Clear documentation of all thresholds and limits
- Supports different battery pack configurations

**Key Configuration Categories:**

| Category | Key Parameters | Default Values |
|----------|---------------|----------------|
| **Pack Configuration** | `BMS_NUM_SLAVES`, `BMS_CELLS_PER_SLAVE` | 2 slaves, 4 cells/slave |
| **Voltage Limits** | `BMS_CELL_VOLT_MIN/MAX`, `BMS_CELL_VOLT_OV/UV_THRESH` | 2800-4200mV, OV=4250mV, UV=2700mV |
| **Temperature Limits** | `BMS_TEMP_MAX_CHARGE/DISCHARGE`, `BMS_TEMP_CRITICAL_HIGH` | Charge: 45C, Discharge: 55C, Critical: 60C |
| **Timing** | `BMS_MEASUREMENT_PERIOD_MS`, `BMS_HEARTBEAT_PERIOD_MS` | 100ms measurement, 1000ms heartbeat |
| **FuSa Parameters** | `BMS_FUSA_FTTI_MS`, `BMS_FUSA_VOLT_PLAUSIBLE_MIN/MAX` | FTTI: 100ms |
| **Feature Enables** | `BMS_ENABLE_LCD`, `BMS_ENABLE_BALANCING`, `BMS_ENABLE_FUSA` | All enabled |

**API Summary:**
```c
// No functions - header-only configuration defines
#define BMS_NUM_SLAVES              2u
#define BMS_CELLS_PER_SLAVE         4u
#define BMS_CELL_VOLT_MIN           2800u
#define BMS_CELL_VOLT_MAX           4200u
#define BMS_FUSA_FTTI_MS            100u
```

---

#### 4.1.2 board_config.h

**Purpose:** Hardware-specific pin assignments and peripheral base addresses for FRDM-KL25Z.

**Design Rationale:**
- Abstracts hardware-specific details from application code
- Enables easy porting to different boards
- Documents pin usage for hardware design reference

**Key Definitions:**

| Peripheral | Pins | Configuration |
|------------|------|---------------|
| **System Clock** | - | 48MHz core, 24MHz bus |
| **RGB LED** | PTB18 (Red), PTB19 (Green), PTD1 (Blue) | Active low |
| **SPI0 (MC33664B)** | PTC5 (SCK), PTC6 (MOSI), PTC7 (MISO), PTC4 (CS) | 1MHz, Mode 0 |
| **UART0 (Debug)** | PTA1 (RX), PTA2 (TX) | 115200 baud, 8N1 |
| **I2C0 (LCD)** | PTE24 (SCL), PTE25 (SDA) | 100kHz standard mode |
| **TPM0 (Fan PWM)** | PTD0 (CH0) | 25kHz PWM |
| **Contactors** | PTE20 (Main), PTE21 (Precharge) | GPIO outputs |
| **Safety Shutdown** | PTE23 | GPIO output |

---

### 4.2 MCAL Layer

#### 4.2.1 GPIO Module (gpio.h / gpio.c)

**Purpose:** Provides register-level GPIO control for all port operations.

**Design Rationale:**
- Direct register access for maximum performance
- Portable API hiding KL25Z-specific register structure
- Support for all GPIO features (direction, pull-up/down, alternate functions)

**Key Features:**
- Support for ports A-E (5 GPIO ports)
- Automatic clock gating enable
- Pin mux configuration for alternate functions
- Atomic set/clear/toggle operations via dedicated registers

**Implementation Details:**
- Uses memory-mapped GPIO_Type structures
- PORT_Type structures for pin control registers
- Clock gating via SIM_SCGC5 register

**API Functions:**

| Function | Description |
|----------|-------------|
| `GPIO_Init(port, pin, direction)` | Initialize pin as input or output |
| `GPIO_SetBit(port, pin)` | Set pin high (PSOR register) |
| `GPIO_ClearBit(port, pin)` | Set pin low (PCOR register) |
| `GPIO_ToggleBit(port, pin)` | Toggle pin state (PTOR register) |
| `GPIO_ReadBit(port, pin)` | Read pin state (PDIR register) |
| `GPIO_ConfigurePull(port, pin, pull)` | Configure pull-up/down |
| `GPIO_SetMux(port, pin, mux)` | Set alternate function |

---

#### 4.2.2 Timer Module (timer.h / timer.c)

**Purpose:** SysTick-based timing services for the entire system.

**Design Rationale:**
- Utilizes ARM Cortex-M0+ SysTick timer (always available)
- 1ms tick resolution for BMS timing requirements
- Wrap-around safe elapsed time calculations

**Key Features:**
- 1ms system tick interrupt
- Blocking millisecond/microsecond delays
- Timeout checking with wrap-around handling
- Elapsed time measurement

**Implementation Details:**
- SysTick configured for 48MHz/1000 = 48000 cycles per tick
- 32-bit tick counter (wraps at ~49.7 days)
- Microsecond delays use CPU cycle counting

**API Functions:**

| Function | Description |
|----------|-------------|
| `TIMER_Init()` | Configure SysTick for 1ms interrupts |
| `TIMER_GetTick()` | Get current tick count (ms) |
| `TIMER_DelayMs(ms)` | Blocking delay in milliseconds |
| `TIMER_DelayUs(us)` | Blocking delay in microseconds |
| `TIMER_IsTimeoutElapsed(start, timeout)` | Check if timeout expired |
| `TIMER_GetElapsed(start)` | Get elapsed time since start |

---

#### 4.2.3 PWM Module (pwm.h / pwm.c)

**Purpose:** TPM-based PWM generation for fan speed control.

**Design Rationale:**
- Uses TPM0 module for hardware PWM generation
- 25kHz frequency optimized for DC fan control
- Edge-aligned PWM for simple duty cycle control

**Key Features:**
- Configurable PWM frequency
- 0-100% duty cycle range
- Runtime frequency changes
- Multi-channel support (TPM0 has 6 channels)

**Implementation Details:**
- TPM clock source: MCGFLLCLK (48MHz)
- MOD register sets frequency: MOD = (48MHz / frequency) - 1
- CnV register sets duty cycle: CnV = (MOD * duty%) / 100
- Edge-aligned high-true PWM mode (MSB=1, ELSB=1)

**API Functions:**

| Function | Description |
|----------|-------------|
| `PWM_Init(config)` | Initialize TPM0 with specified configuration |
| `PWM_SetDutyCycle(channel, percent)` | Set duty cycle 0-100% |
| `PWM_GetDutyCycle(channel)` | Get current duty cycle |
| `PWM_Start()` | Enable PWM output |
| `PWM_Stop()` | Disable PWM output |
| `PWM_SetFrequency(hz)` | Change PWM frequency |

---

#### 4.2.4 I2C Module (i2c.h / i2c.c)

**Purpose:** I2C master driver for LCD communication.

**Design Rationale:**
- Standard I2C protocol implementation
- Master-only mode (BMS is always master)
- Support for combined write-read transactions

**Key Features:**
- 100kHz standard mode operation
- 7-bit addressing
- Repeated start support
- Timeout protection on all operations

**Implementation Details:**
- Uses I2C0 module on PTE24 (SCL) and PTE25 (SDA)
- Baud rate divider calculated for 24MHz bus clock
- Interrupt flag polling for byte completion
- ACK/NACK detection for error handling

**API Functions:**

| Function | Description |
|----------|-------------|
| `I2C_Init(config)` | Initialize I2C0 peripheral |
| `I2C_Write(addr, data, len)` | Write data to slave |
| `I2C_Read(addr, data, len)` | Read data from slave |
| `I2C_WriteRead(addr, tx, tx_len, rx, rx_len)` | Combined transaction |
| `I2C_WriteByte(addr, data)` | Write single byte |
| `I2C_IsBusy()` | Check if bus is busy |
| `I2C_Reset()` | Reset I2C peripheral |

---

#### 4.2.5 SPI Module (spi_mc33664b.h / spi_mc33664b.c)

**Purpose:** SPI driver optimized for MC33664B transceiver communication.

**Design Rationale:**
- Mode 0 (CPOL=0, CPHA=0) as required by MC33664B
- Software-controlled CS for precise timing
- Full-duplex operation for request-response protocol

**Key Features:**
- 1MHz SPI clock (MC33664B supports up to 4MHz)
- MSB-first bit order
- GPIO-controlled chip select
- Timeout protection

**Implementation Details:**
- Uses SPI0 module on Port C
- Baud rate: 24MHz / ((SPPR+1) * 2^(SPR+1)) = 24MHz / 24 = 1MHz
- Manual CS control for wake-up pulse generation
- Flag polling: SPTEF for TX ready, SPRF for RX complete

**API Functions:**

| Function | Description |
|----------|-------------|
| `SPI_MC33664B_Init(config)` | Initialize SPI0 for MC33664B |
| `SPI_MC33664B_TransferData(tx, rx, len)` | Full-duplex transfer |
| `SPI_MC33664B_CS_Assert()` | Assert chip select (low) |
| `SPI_MC33664B_CS_Deassert()` | Deassert chip select (high) |
| `SPI_MC33664B_ReadByte()` | Read single byte (sends 0xFF) |
| `SPI_MC33664B_WriteByte(data)` | Write single byte |

---

#### 4.2.6 UART Module (uart_kl25z.h / uart_kl25z.c)

**Purpose:** Debug UART interface connected to OpenSDA for serial output.

**Design Rationale:**
- Standard debug interface for development
- Printf-style output support
- Non-blocking transmit for minimal timing impact

**Key Features:**
- 115200 baud, 8N1 configuration
- Transmit string and buffer functions
- Integer and hex print helpers
- Receive with timeout support

**Implementation Details:**
- Uses UART0 on PTA1 (RX) and PTA2 (TX)
- Baud rate: 48MHz / (16 * 26) = 115384 baud (0.16% error)
- Fine adjust (BRFD) for improved accuracy
- Connected to OpenSDA for USB virtual COM port

**API Functions:**

| Function | Description |
|----------|-------------|
| `UART_Init(config)` | Initialize UART0 peripheral |
| `UART_TransmitByte(data)` | Send single byte (blocking) |
| `UART_TransmitString(str)` | Send null-terminated string |
| `UART_TransmitBuffer(data, len)` | Send buffer |
| `UART_ReceiveByte(data, timeout)` | Receive with timeout |
| `UART_DataAvailable()` | Check if data ready |
| `UART_PrintInt(value)` | Print integer as decimal |
| `UART_PrintHex(value)` | Print as hex (0xXXXXXXXX) |

---

### 4.3 HAL Layer

#### 4.3.1 MC33771 Driver (mc33771_driver.h / mc33771_driver.c)

**Purpose:** High-level driver for NXP MC33771C battery cell controller via MC33664B transceiver.

**Design Rationale:**
- Abstracts complex MC33771C register protocol
- Encapsulates SPI communication and CRC handling
- Provides measurement and balancing control API

**Key Features:**
- Cell voltage measurement (up to 14 cells per slave)
- Temperature measurement via analog inputs
- Passive cell balancing control
- Fault detection and reporting
- Daisy chain support (multiple slaves)

**Implementation Details:**

**Communication Protocol:**
```
Master -> MC33664B (SPI) -> MC33771C chain (TPL)

Frame Format (40-bit):
┌──────────┬──────────┬──────────┬──────────┬──────────┐
│ Byte 0   │ Byte 1   │ Byte 2   │ Byte 3   │ Byte 4   │
│ DevAddr  │ CMD+Reg  │ Data Hi  │ Data Lo  │  CRC-8   │
└──────────┴──────────┴──────────┴──────────┴──────────┘
```

**Register Map:**
- 0x12-0x1F: Cell voltage registers (CELL1-CELL14)
- 0x20-0x26: Analog input registers (AN0-AN6) for temperature
- 0x28-0x2B: Cell balancing configuration
- 0x02-0x04: Status/fault registers

**ADC Conversion:**
- 16-bit resolution, 5V reference
- Voltage (mV) = raw_ADC * 5000 / 65536

**API Functions:**

| Function | Description |
|----------|-------------|
| `SlaveIF_Init()` | Initialize SPI and configure all slaves |
| `SlaveIF_RequestMeasurements(slave_id)` | Trigger measurement on slave |
| `SlaveIF_GetSlaveData(slave_id, data)` | Read all measurements |
| `SlaveIF_ReadCellVoltages(slave_id, voltages)` | Read cell voltages |
| `SlaveIF_ReadTemperatures(slave_id, temps)` | Read temperatures |
| `SlaveIF_EnableBalancing(slave_id, config)` | Enable cell balancing |
| `SlaveIF_DisableBalancing(slave_id)` | Disable balancing |
| `SlaveIF_ReadFaultStatus(slave_id, status)` | Read fault flags |
| `SlaveIF_ClearFaults(slave_id)` | Clear fault flags |
| `SlaveIF_WakeUp()` | Wake slaves from sleep |
| `SlaveIF_Sleep()` | Put slaves to sleep |

---

#### 4.3.2 LCD Interface (lcd_if.h / lcd_if.c)

**Purpose:** Driver for 16x2 character LCD with PCF8574 I2C backpack.

**Design Rationale:**
- Standard HD44780 controller protocol
- I2C interface minimizes pin usage
- 4-bit mode saves I/O pins on expander

**Key Features:**
- 16 columns x 2 rows display
- Backlight control
- Custom character support (8 characters)
- Cursor and blink control
- Display scrolling

**Implementation Details:**

**PCF8574 Pin Mapping:**
```
P0 = RS (Register Select)
P1 = RW (Read/Write, always 0 for write)
P2 = EN (Enable strobe)
P3 = Backlight
P4-P7 = D4-D7 (4-bit data)
```

**Initialization Sequence:**
1. Wait 50ms for power-up
2. Send 0x30 three times (force 8-bit mode)
3. Send 0x20 (switch to 4-bit mode)
4. Configure: 4-bit, 2 lines, 5x8 font
5. Display on, cursor off
6. Clear display
7. Entry mode: increment, no shift

**API Functions:**

| Function | Description |
|----------|-------------|
| `LCD_Init()` | Initialize LCD with default address (0x27) |
| `LCD_InitWithAddress(addr)` | Initialize with specific I2C address |
| `LCD_Clear()` | Clear display |
| `LCD_Home()` | Move cursor to (0,0) |
| `LCD_SetCursor(row, col)` | Position cursor |
| `LCD_PrintChar(c)` | Print single character |
| `LCD_Print(str)` | Print string |
| `LCD_PrintAt(row, col, str)` | Print at position |
| `LCD_PrintInt(value)` | Print integer |
| `LCD_BacklightOn/Off()` | Control backlight |
| `LCD_DisplayOn/Off()` | Control display |
| `LCD_CreateChar(loc, charmap)` | Define custom character |

---

### 4.4 Services Layer

#### 4.4.1 BMS Database (bms_database.h / bms_database.c)

**Purpose:** Central repository for all BMS measurement data.

**Design Rationale:**
- Single source of truth for all modules
- Encapsulates data storage and access
- Provides data aggregation functions (min/max)
- Tracks data validity and timestamps

**Key Features:**
- Cell voltage storage for all cells
- Temperature storage for all sensors
- System status tracking
- Data freshness timestamps
- Min/max voltage and temperature queries

**Data Structures:**
```c
typedef struct {
    uint16_t cell_voltages_mV[BMS_TOTAL_CELLS];
    int8_t   cell_temperatures_C[BMS_TOTAL_CELLS];
    uint32_t last_update_ms;
    bool     data_valid;
} BMS_CellData_t;

typedef struct {
    bool system_ok;
    bool overvoltage_fault;
    bool undervoltage_fault;
    bool overtemp_fault;
    bool comm_fault;
    uint8_t active_fault_count;
} BMS_SystemStatus_t;
```

**API Functions:**

| Function | Description |
|----------|-------------|
| `BMS_DB_Init()` | Initialize database |
| `BMS_DB_UpdateCellVoltages(slave, voltages)` | Store voltage data |
| `BMS_DB_UpdateTemperatures(slave, temps)` | Store temperature data |
| `BMS_DB_GetCellVoltage(index)` | Get single cell voltage |
| `BMS_DB_GetCellTemperature(index)` | Get single temperature |
| `BMS_DB_GetMaxCellVoltage()` | Get maximum voltage |
| `BMS_DB_GetMinCellVoltage()` | Get minimum voltage |
| `BMS_DB_GetMaxTemperature()` | Get maximum temperature |
| `BMS_DB_SetSystemStatus(status)` | Update system status |
| `BMS_DB_GetSystemStatus(status)` | Read system status |
| `BMS_DB_GetAllData()` | Get pointer to all data |

---

#### 4.4.2 Debug Info Manager (debug_info_mgr.h / debug_info_mgr.c)

**Purpose:** Centralized debug logging service over UART.

**Design Rationale:**
- Consistent log message format across all modules
- Severity levels for filtering
- Timestamps for event correlation
- Easily disable in production builds

**Key Features:**
- Multiple log levels (INFO, WARNING, ERROR, DATA)
- Automatic timestamp prefix
- Formatted data logging
- UART output to OpenSDA debug port

**Log Format:**
```
[timestamp_ms][LEVEL] message
Example: [12345][INFO] BMS initialization complete
```

**API Functions:**

| Function | Description |
|----------|-------------|
| `DebugMgr_Init()` | Initialize UART for debug output |
| `DebugMgr_LogInfo(msg)` | Log informational message |
| `DebugMgr_LogWarning(msg)` | Log warning message |
| `DebugMgr_LogError(msg)` | Log error message |
| `DebugMgr_LogData(label, value)` | Log named data value |

---

#### 4.4.3 Watchdog Manager (wdg_mgr.h / wdg_mgr.c)

**Purpose:** Watchdog supervision for system health monitoring.

**Design Rationale:**
- Prevents system lockup on software faults
- Integrates with FuSa supervisor
- Supports multiple task supervision

**Key Features:**
- Configurable timeout period
- Task registration for supervised execution
- Refresh/kick function for main loop

**Implementation Notes:**
- Current implementation is software-based placeholder
- Production version would use KL25Z COP (Computer Operating Properly) watchdog

**API Functions:**

| Function | Description |
|----------|-------------|
| `WDG_Init(timeout_ms)` | Initialize watchdog with timeout |
| `WDG_Refresh()` | Refresh/kick watchdog |
| `WDG_RegisterTask(name)` | Register task for supervision |

---

### 4.5 Application Layer

#### 4.5.1 FuSa Supervisor (fusa_supervisor.h / fusa_supervisor.c)

**Purpose:** Functional Safety supervisor implementing ISO 26262 ASIL-C safety monitoring.

**Design Rationale:**
- Centralized safety monitoring and enforcement
- State machine for safe state transitions
- Comprehensive fault detection and logging
- Prevents unsafe operations

**Key Features:**

**Safety States:**
```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌───────────┐
│   INIT   │───►│  NORMAL  │───►│ DEGRADED │───►│   SAFE    │
└──────────┘    └─────┬────┘    └─────┬────┘    └─────┬─────┘
                      │               │               │
                      │               │               ▼
                      │               │         ┌───────────┐
                      └───────────────┴────────►│ EMERGENCY │
                                                └───────────┘
```

**Fault Detection:**
1. **Voltage Plausibility:** Check sensor range (1500-5500mV)
2. **Temperature Plausibility:** Check sensor range (-50 to +150C)
3. **Voltage Gradient:** Rate of change < 100mV/100ms
4. **Temperature Gradient:** Rate of change < 5C/s
5. **Critical Limits:** OV > 4300mV, UV < 2500mV, OT > 60C
6. **Data Freshness:** Data age < 500ms
7. **Alive Supervision:** Periodic alive indication
8. **RAM Integrity:** Checkerboard pattern test

**Fault Types:**
| Fault | Severity | Action |
|-------|----------|--------|
| `FUSA_FAULT_VOLTAGE_PLAUSIBILITY` | Degraded | Limit operations |
| `FUSA_FAULT_TEMP_PLAUSIBILITY` | Degraded | Limit operations |
| `FUSA_FAULT_VOLTAGE_GRADIENT` | Warning | Log only |
| `FUSA_FAULT_VOLTAGE_CRITICAL` | Critical | Safe state |
| `FUSA_FAULT_TEMP_CRITICAL` | Critical | Safe state |
| `FUSA_FAULT_RAM_CORRUPTION` | Critical | Safe state |

**Operation Permissions:**
| State | Contactors | Charging | Discharging |
|-------|------------|----------|-------------|
| NORMAL | Allowed | Allowed* | Allowed* |
| DEGRADED | Allowed | Blocked | Allowed |
| SAFE | Blocked | Blocked | Blocked |
| EMERGENCY | Blocked | Blocked | Blocked |

*Subject to temperature and voltage limits

**API Functions:**

| Function | Description |
|----------|-------------|
| `FuSa_Init()` | Initialize supervisor, run self-tests |
| `FuSa_PerformSafetyChecks()` | Execute all safety checks (cyclic) |
| `FuSa_GetSafetyStatus()` | Get overall safety status |
| `FuSa_GetDetailedStatus(status)` | Get detailed status structure |
| `FuSa_GetState()` | Get current state machine state |
| `FuSa_IsOperationAllowed(op)` | Check if operation is permitted |
| `FuSa_RequestSafeState(fault)` | Force safe state transition |
| `FuSa_AttemptRecovery()` | Attempt to recover from safe state |
| `FuSa_ClearFault(type)` | Clear specific fault |
| `FuSa_GetFaultEntry(index, entry)` | Read fault log entry |
| `FuSa_AliveIndication()` | Signal task is alive |
| `FuSa_CheckRAMIntegrity()` | Perform RAM test |
| `FuSa_EmergencyShutdown()` | Enter emergency state (no return) |

---

#### 4.5.2 Cell Balancing Manager (cell_balancing_mgr.h / cell_balancing_mgr.c)

**Purpose:** Passive cell balancing algorithm for pack equalization.

**Design Rationale:**
- Passive balancing via MC33771C bleed resistors
- Safety-first approach with multiple inhibit conditions
- State machine for controlled balancing cycles

**Balancing Strategy:**
1. Find minimum cell voltage in pack
2. Identify cells exceeding minimum + threshold (20mV)
3. Enable balancing on high cells
4. Monitor until delta < hysteresis (10mV)
5. Enter cooldown period before next cycle

**State Machine:**
```
┌──────────┐    ┌────────────┐    ┌──────────┐    ┌──────────┐
│   IDLE   │───►│ EVALUATING │───►│  ACTIVE  │───►│ COOLDOWN │
└────┬─────┘    └─────┬──────┘    └─────┬────┘    └────┬─────┘
     │                │                 │              │
     │                ▼                 ▼              │
     │         ┌───────────┐    ┌───────────┐         │
     └────────►│ INHIBITED │◄───┤ (inhibit) │◄────────┘
               └───────────┘    └───────────┘
```

**Inhibit Conditions:**
| Condition | Flag | Description |
|-----------|------|-------------|
| Voltage Low | 0x01 | Min cell < 3200mV |
| Voltage High | 0x02 | Max cell > 4150mV |
| Temp Low | 0x04 | Any cell < 0C |
| Temp High | 0x08 | Max temp > 45C |
| Charging | 0x10 | Active charging detected |
| Comm Fault | 0x20 | Communication error |
| Safety | 0x40 | FuSa supervisor inhibit |

**Configuration:**
```c
BALANCE_START_THRESHOLD_MV    = 20mV   // Start when delta > 20mV
BALANCE_STOP_HYSTERESIS_MV    = 10mV   // Stop when delta < 10mV
BALANCE_MAX_TIME_S            = 60s    // Max balance time per cycle
BALANCE_MIN_VOLTAGE_MV        = 3200mV // Don't balance below this
BALANCE_COOLDOWN_MS           = 5000ms // Cooldown between cycles
```

**API Functions:**

| Function | Description |
|----------|-------------|
| `CellBalance_Init()` | Initialize balancing manager |
| `CellBalance_Update()` | Periodic update (call from main loop) |
| `CellBalance_IsBalancing()` | Check if balancing is active |
| `CellBalance_GetState()` | Get current state |
| `CellBalance_GetStatus(status)` | Get detailed status |
| `CellBalance_SetEnabled(enable)` | Enable/disable balancing |
| `CellBalance_IsEnabled()` | Check if enabled |
| `CellBalance_EmergencyStop()` | Immediately stop all balancing |
| `CellBalance_SetExternalInhibit(inhibit)` | External inhibit control |
| `CellBalance_GetMaxDelta()` | Get max cell-to-cell delta |

---

#### 4.5.3 Diagnostics Manager (diagnostics_mgr.h / diagnostics_mgr.c)

**Purpose:** Fault detection and diagnostic status management.

**Design Rationale:**
- Dedicated fault monitoring separate from FuSa
- Simpler fault detection for operational warnings
- Provides fault count for system status display

**Key Features:**
- Overvoltage detection
- Undervoltage detection
- Overtemperature detection
- Communication error tracking

**Fault Types:**
| Fault | Threshold | Check Function |
|-------|-----------|----------------|
| Overvoltage | > BMS_CELL_VOLT_MAX (4200mV) | `Diagnostics_CheckVoltages()` |
| Undervoltage | < BMS_CELL_VOLT_MIN (2800mV) | `Diagnostics_CheckVoltages()` |
| Overtemperature | > BMS_TEMP_MAX_DISCHARGE (55C) | `Diagnostics_CheckTemperatures()` |
| Communication | Slave not responding | (External trigger) |

**API Functions:**

| Function | Description |
|----------|-------------|
| `Diagnostics_Init()` | Initialize diagnostics |
| `Diagnostics_CheckVoltages()` | Check OV/UV conditions |
| `Diagnostics_CheckTemperatures()` | Check overtemperature |
| `Diagnostics_GetFaultCount()` | Get count of active faults |
| `Diagnostics_IsFaultActive(fault)` | Check specific fault status |

---

#### 4.5.4 Thermal Manager (thermal_mgr.h / thermal_mgr.c)

**Purpose:** Coordinates thermal monitoring and fan control.

**Design Rationale:**
- Separation between thermal monitoring and fan actuation
- Centralized thermal status management
- Interface between database and fan control

**Key Features:**
- Reads temperatures from database
- Triggers fan control updates
- Provides max temperature query

**API Functions:**

| Function | Description |
|----------|-------------|
| `ThermalMgr_Init()` | Initialize thermal manager |
| `ThermalMgr_Update()` | Periodic update (triggers fan control) |
| `ThermalMgr_GetMaxTemp()` | Get maximum pack temperature |

---

#### 4.5.5 Fan Control (fan_control.h / fan_control.c)

**Purpose:** PWM-based cooling fan speed control.

**Design Rationale:**
- Temperature-proportional speed control
- Hysteresis to prevent rapid on/off cycling
- Configurable temperature thresholds

**Control Algorithm:**
```
                    ┌─────────────────────────────────────┐
                    │         Fan Speed Curve             │
    100% ─ ─ ─ ─ ─ ─│─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┌──┤
                    │                              ┌──┘  │
                    │                         ┌────┘     │
                    │                    ┌────┘          │
     30% ─ ─ ─ ─ ─ ─│─ ─ ─ ─ ─ ─ ─ ─┌───┘               │
                    │               │                    │
      0% ────────────────────────────────────────────────┤
                    35   40              60   Temperature (C)
                    OFF  ON
```

**Hysteresis:**
- Fan ON threshold: 40C
- Fan OFF threshold: 35C
- Prevents cycling when temperature hovers near threshold

**Speed Mapping:**
- Below 40C: 0% (fan off)
- 40C: 30% (minimum active speed)
- 40-60C: Linear 30-100%
- Above 60C: 100%

**API Functions:**

| Function | Description |
|----------|-------------|
| `FanControl_Init()` | Initialize PWM for fan |
| `FanControl_Update(temp)` | Update fan speed based on temperature |
| `FanControl_SetSpeed(percent)` | Set fan speed directly |
| `FanControl_GetSpeed()` | Get current fan speed |

---

#### 4.5.6 Battery Status Monitor (battery_status_mon.h / battery_status_mon.c)

**Purpose:** State of Charge (SOC) estimation and pack voltage calculation.

**Design Rationale:**
- Simple voltage-based SOC for demonstration
- Production systems would use coulomb counting + OCV

**SOC Estimation:**
```
SOC = (V_avg - V_min) / (V_max - V_min) * 100%

Where:
  V_avg = Average cell voltage
  V_min = 2800mV (0% SOC)
  V_max = 4200mV (100% SOC)
```

**Pack Voltage:**
Sum of all cell voltages (series configuration assumed)

**API Functions:**

| Function | Description |
|----------|-------------|
| `BatteryStatus_Init()` | Initialize status monitor |
| `BatteryStatus_Update()` | Update SOC calculation |
| `BatteryStatus_GetSOC()` | Get SOC percentage (0-100) |
| `BatteryStatus_GetPackVoltage()` | Get total pack voltage (mV) |

---

### 4.6 Main Application (main.c)

**Purpose:** System entry point, initialization orchestration, and main control loop.

**Design Rationale:**
- Clear initialization sequence
- Deterministic periodic execution
- Centralized error handling for init failures

**Execution Flow:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         INITIALIZATION                                   │
├─────────────────────────────────────────────────────────────────────────┤
│  1. TIMER_Init()        - System tick (1ms)                             │
│  2. GPIO_Init()         - LED outputs                                   │
│  3. DebugMgr_Init()     - Debug UART (115200)                          │
│  4. BMS_DB_Init()       - Data storage                                  │
│  5. WDG_Init()          - Watchdog (if enabled)                         │
│  6. SlaveIF_Init()      - MC33771 via MC33664B                         │
│  7. LCD_Init()          - Display (if enabled)                          │
│  8. Diagnostics_Init()  - Fault detection                               │
│  9. FuSa_Init()         - Safety supervisor (if enabled)                │
│ 10. BatteryStatus_Init() - SOC estimation                               │
│ 11. FanControl_Init()   - PWM for fan                                   │
│ 12. ThermalMgr_Init()   - Thermal coordination                          │
│ 13. CellBalance_Init()  - Cell balancing (if enabled)                   │
├─────────────────────────────────────────────────────────────────────────┤
│                         MAIN LOOP (10ms cycle)                          │
├─────────────────────────────────────────────────────────────────────────┤
│  Every 100ms (BMS_MEASUREMENT_PERIOD_MS):                               │
│    - SlaveIF_RequestMeasurements() for each slave                       │
│    - SlaveIF_GetSlaveData()                                             │
│    - BMS_DB_UpdateCellVoltages()                                        │
│    - BMS_DB_UpdateTemperatures()                                        │
│    - Diagnostics_CheckVoltages()                                        │
│    - Diagnostics_CheckTemperatures()                                    │
│    - FuSa_PerformSafetyChecks()                                         │
│    - BatteryStatus_Update()                                             │
│    - ThermalMgr_Update()                                                │
│    - CellBalance_Update()                                               │
│    - BMS_UpdateLCD()                                                    │
│    - Toggle heartbeat LED                                               │
│                                                                         │
│  Every 1000ms (BMS_HEARTBEAT_PERIOD_MS):                               │
│    - Log all cell voltages                                              │
│    - Log min/max voltage range                                          │
│    - Log temperatures                                                   │
│    - Log fan speed                                                      │
│    - Log SOC and pack voltage                                           │
│    - Log fault status                                                   │
│                                                                         │
│  Every cycle:                                                           │
│    - WDG_Refresh()                                                      │
│    - TIMER_DelayMs(10) - Prevent busy-waiting                          │
└─────────────────────────────────────────────────────────────────────────┘
```

**Error Handling:**
- Initialization failure: Enter error loop with red LED blinking
- Non-critical failures (LCD): Log warning, continue operation
- Runtime faults: Handled by Diagnostics and FuSa modules

---

## 5. Safety Architecture (ISO 26262 FuSa)

### 5.1 Safety Concept Overview

The BMS implements safety mechanisms aligned with **ISO 26262 ASIL-C** requirements:

| Safety Requirement | Implementation |
|-------------------|----------------|
| Fault Detection Time | FTTI = 100ms (configurable) |
| Single Point Fault Detection | Plausibility checks, gradient monitoring |
| Latent Fault Detection | RAM integrity test, alive supervision |
| Safe State Definition | Open contactors, disable operations |
| Fault Reaction | State machine with defined transitions |

### 5.2 Safety Goals

| Safety Goal | ASIL | Description |
|-------------|------|-------------|
| SG1 | ASIL-C | Prevent cell overvoltage (thermal runaway risk) |
| SG2 | ASIL-C | Prevent cell undervoltage (damage, capacity loss) |
| SG3 | ASIL-C | Prevent overtemperature (thermal runaway risk) |
| SG4 | ASIL-B | Prevent uncontrolled contactor operation |

### 5.3 Diagnostic Coverage

| Mechanism | Detection Target | Coverage |
|-----------|-----------------|----------|
| Plausibility Check | Sensor failure | High (90%) |
| Gradient Monitoring | Sensor drift, sudden failure | Medium (60%) |
| CRC Verification | Communication corruption | High (99.6%) |
| RAM Test | Memory corruption | High (90%) |
| Alive Supervision | Software lockup | High (90%) |
| Redundancy Check | Systematic errors | Medium (60%) |

### 5.4 Safe State Definition

**Safe State:** System configuration that prevents hazardous conditions:

1. **Open all contactors** - Disconnect battery from load/charger
2. **Disable cell balancing** - Prevent additional heat generation
3. **Block charging/discharging** - Prevent further energy flow
4. **Assert safety shutdown output** - Signal external safety systems
5. **Log fault information** - Enable post-incident analysis

### 5.5 Fault Tolerance Time Interval (FTTI)

```
FTTI = 100ms

Timeline:
├── 0ms    Fault occurs
├── 100ms  Maximum detection time (BMS_MEASUREMENT_PERIOD_MS)
├── +10ms  State transition time
├── +5ms   Contactor opening time (mechanical)
└── 115ms  System in safe state
```

### 5.6 Safety State Machine

```
                    ┌─────────────────────────────────────────────┐
                    │                                             │
                    ▼                                             │
             ┌──────────┐                                         │
    Power ──►│   INIT   │                                         │
    On       └────┬─────┘                                         │
                  │ Self-test passed                              │
                  ▼                                               │
             ┌──────────┐    Warning     ┌──────────┐             │
             │  NORMAL  │◄──────────────►│  (same)  │             │
             └────┬─────┘                └──────────┘             │
                  │                                               │
                  │ Degraded fault                                │
                  ▼                                               │
             ┌──────────┐    Cleared     ┌──────────┐             │
             │ DEGRADED │───────────────►│  NORMAL  │             │
             └────┬─────┘                └──────────┘             │
                  │                                               │
                  │ Critical fault                                │
                  ▼                                               │
             ┌──────────┐    Recovery    ┌──────────┐             │
             │   SAFE   │───────────────►│  NORMAL  │─────────────┘
             └────┬─────┘  (if cleared)  └──────────┘
                  │
                  │ Unrecoverable fault
                  ▼
             ┌───────────┐
             │ EMERGENCY │  ──► Infinite loop (requires power cycle)
             └───────────┘
```

---

## 6. Data Flow

### 6.1 Measurement Data Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    MEASUREMENT DATA FLOW                                │
└─────────────────────────────────────────────────────────────────────────┘

  MC33771C Slave 0          MC33771C Slave 1
  ┌─────────────┐           ┌─────────────┐
  │ Cell 1-4    │           │ Cell 5-8    │
  │ Temp AN0-1  │           │ Temp AN0-1  │
  └──────┬──────┘           └──────┬──────┘
         │ TPL                     │ TPL
         └───────────┬─────────────┘
                     │
                     ▼
              ┌──────────────┐
              │   MC33664B   │  SPI-to-TPL Bridge
              │  Transceiver │
              └──────┬───────┘
                     │ SPI (1MHz)
                     ▼
              ┌──────────────┐
              │  SPI Driver  │  MCAL Layer
              │ (spi_mc33664b)│
              └──────┬───────┘
                     │
                     ▼
              ┌──────────────┐
              │ MC33771 Drv  │  HAL Layer
              │ (SlaveIF)    │  - CRC check
              └──────┬───────┘  - ADC conversion
                     │
                     ▼
              ┌──────────────┐
              │ BMS Database │  Services Layer
              │ (bms_database)│ - Central storage
              └──────┬───────┘  - Min/Max calc
                     │
         ┌───────────┼───────────┬───────────┬───────────┐
         ▼           ▼           ▼           ▼           ▼
   ┌──────────┐┌──────────┐┌──────────┐┌──────────┐┌──────────┐
   │Diagnostics││  FuSa   ││ Thermal  ││ Battery  ││   Cell   │
   │  Manager ││Supervisor││ Manager  ││ Status   ││ Balance  │
   └──────────┘└──────────┘└──────────┘└──────────┘└──────────┘
```

### 6.2 Control Flow (Balancing)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    BALANCING CONTROL FLOW                               │
└─────────────────────────────────────────────────────────────────────────┘

              ┌──────────────┐
              │ BMS Database │  1. Read cell voltages
              └──────┬───────┘
                     │
                     ▼
              ┌──────────────┐
              │ Cell Balance │  2. Evaluate need
              │   Manager    │  3. Check inhibits
              │              │  4. Build balance mask
              └──────┬───────┘
                     │
                     ▼
              ┌──────────────┐
              │ MC33771 Drv  │  5. Write CB_CFG registers
              │ EnableBalance│
              └──────┬───────┘
                     │ SPI
                     ▼
              ┌──────────────┐
              │   MC33771C   │  6. Internal MOSFETs
              │ Bleed Resist │     discharge high cells
              └──────────────┘
```

### 6.3 Debug Data Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    DEBUG DATA FLOW                                      │
└─────────────────────────────────────────────────────────────────────────┘

  All Modules                       Debug Info Manager         UART
  ┌───────────┐                     ┌───────────────┐         ┌────────┐
  │ FuSa      │──LogError()───────►│               │         │        │
  │ Diagnostics│──LogWarning()────►│ Format with   │────────►│ UART0  │
  │ CellBalance│──LogInfo()───────►│ timestamp     │ 115200  │(OpenSDA)│
  │ main()    │──LogData()────────►│ and level     │         │        │
  └───────────┘                     └───────────────┘         └────┬───┘
                                                                   │ USB
                                                                   ▼
                                                              ┌────────┐
                                                              │  PC    │
                                                              │Terminal│
                                                              └────────┘
```

---

## 7. Key Design Decisions

### 7.1 Architecture Decisions

| Decision | Rationale | Trade-offs |
|----------|-----------|------------|
| **4-Layer Architecture** | Clear separation of concerns, easier testing, portable | Slight overhead for simple operations |
| **Centralized Database** | Single source of truth, simplified data access | Memory overhead, potential bottleneck |
| **Static Memory Allocation** | Deterministic behavior, no heap fragmentation | Fixed limits, requires careful sizing |
| **Polling-based Design** | Simpler than interrupt-driven, predictable timing | Less efficient CPU usage |

### 7.2 Safety Decisions

| Decision | Rationale | ISO 26262 Alignment |
|----------|-----------|---------------------|
| **Software Watchdog** | Detect software lockup | Part 6: Software component design |
| **RAM Integrity Test** | Detect memory corruption | Part 5: Hardware-software interface |
| **Plausibility Checks** | Detect sensor failures | Part 6: Safety mechanisms |
| **Gradient Monitoring** | Detect sudden changes | Part 6: Temporal plausibility |
| **Safe State Definition** | Known-safe configuration | Part 4: Technical safety concept |
| **Fault Debouncing** | Avoid false positives | Part 6: Diagnostic accuracy |

### 7.3 Communication Decisions

| Decision | Rationale |
|----------|-----------|
| **SPI for MC33664B** | Required by transceiver, low pin count |
| **1MHz SPI Clock** | Conservative for reliability (max 4MHz) |
| **Software CS Control** | Precise timing for wake-up sequences |
| **CRC-8 Verification** | Required by MC33771 protocol, error detection |
| **I2C for LCD** | Standard interface, minimal pins |
| **UART Debug at 115200** | Standard rate, compatible with OpenSDA |

### 7.4 Timing Decisions

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| **Measurement Period** | 100ms | Balances responsiveness with CPU load |
| **Heartbeat Period** | 1000ms | Human-readable logging frequency |
| **FTTI** | 100ms | Matches measurement period for ASIL-C |
| **Balance Cooldown** | 5000ms | Allows temperature stabilization |
| **Data Stale Timeout** | 500ms | 5x measurement period margin |
| **Watchdog Timeout** | 200ms | 2x measurement period margin |

### 7.5 Memory Decisions

| Category | Size | Notes |
|----------|------|-------|
| **Cell Data Array** | 16 bytes | 8 cells x 2 bytes voltage |
| **Temperature Array** | 8 bytes | 8 cells x 1 byte |
| **Fault Log** | 256 bytes | 16 entries x 16 bytes |
| **RAM Test Area** | 32 bytes | Dedicated for checkerboard test |
| **Stack** | ~2KB | Estimated for all function calls |
| **Total RAM** | ~4KB | Well within 16KB available |

### 7.6 Error Handling Strategy

| Error Type | Handling |
|------------|----------|
| **Init Failure** | Red LED blink loop (cannot continue) |
| **Non-critical Failure** | Log warning, continue operation (e.g., LCD) |
| **Communication Timeout** | Increment counter, set stale flag |
| **CRC Error** | Retry once, then fail operation |
| **Plausibility Failure** | Debounce, then enter degraded state |
| **Critical Fault** | Immediate safe state transition |

---

## Appendix A: Build Configuration

### Recommended Compiler Settings

```
Target: ARM Cortex-M0+
Optimization: -O2 (Release), -O0 (Debug)
Warnings: -Wall -Wextra
Standard: C99 or C11
```

### Feature Enables (bms_config.h)

```c
#define BMS_ENABLE_LCD           1    // LCD display
#define BMS_ENABLE_BALANCING     1    // Cell balancing
#define BMS_ENABLE_FUSA          1    // Safety supervisor
#define BMS_ENABLE_WATCHDOG      1    // Watchdog timer
#define BMS_ENABLE_DEBUG_UART    1    // Debug output
```

---

## Appendix B: Pin Assignment Summary

| Pin | Function | Module | Direction |
|-----|----------|--------|-----------|
| PTA1 | UART0_RX | Debug | Input |
| PTA2 | UART0_TX | Debug | Output |
| PTB18 | Red LED | Status | Output |
| PTB19 | Green LED | Status | Output |
| PTC4 | SPI0_CS | MC33664B | Output |
| PTC5 | SPI0_SCK | MC33664B | Output |
| PTC6 | SPI0_MOSI | MC33664B | Output |
| PTC7 | SPI0_MISO | MC33664B | Input |
| PTD0 | TPM0_CH0 | Fan PWM | Output |
| PTD1 | Blue LED | Status | Output |
| PTE20 | Main Contactor | Safety | Output |
| PTE21 | Precharge Contactor | Safety | Output |
| PTE23 | Safety Shutdown | Safety | Output |
| PTE24 | I2C0_SCL | LCD | Bidirectional |
| PTE25 | I2C0_SDA | LCD | Bidirectional |

---

## Appendix C: References

1. NXP FRDM-KL25Z User Guide
2. Kinetis KL25 Sub-Family Reference Manual
3. NXP MC33771C Datasheet
4. NXP MC33664B Datasheet
5. ISO 26262:2018 Road vehicles - Functional safety
6. HD44780 LCD Controller Datasheet
7. PCF8574 I2C I/O Expander Datasheet

---

*Document generated for EV-BMS Modular System Demo*
*NXP EVB Reference Implementation*
