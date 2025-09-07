#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/*
# I²C Interface Specification – TI Fuel Gauge (e.g., bq27426)

## I²C Addressing
- **7-bit address**: `0x55` (binary: `1010101`)
- **8-bit addresses**:
  - Write: `0xAA`
  - Read: `0xAB`

## Supported I²C Modes
- **Standard Mode**: 100 kHz
- **Fast Mode**: 400 kHz

## Supported Command Types
- **1-Byte Write**
- **Quick Read**
- **1-Byte Read**
- **Incremental Read**
- **Incremental Write**

## Unsupported Operations
- **Write to Read-Only Address**: Results in NACK after data byte
- **Read from Address > 0x6B**: Results in NACK

## Timing Requirements
- **Bus-Free Time (`t_BUF`)**: ≥ 66 µs between packets at 400 kHz
- **Read-Modify-Write Cycles**: Insert 66 µs between consecutive write or read/write operations
- **Subcommand Result Wait**:
  - Requires ≥ 2 seconds after issuing a control command to read back valid data
- **Command Rate Limit**:
  - No more than 2 standard commands/second
  - Prevents watchdog reset due to command flooding

## I²C Clock Stretching
- **SLEEP Mode**: ≤ 100 µs
- **ACTIVE Modes (Initialization, Normal)**: ≤ 4 ms during packet handling

## I²C Bus Timeout
- If SDA or SCL is held low for **≥ 2 seconds**, the chip:
  - Releases the bus
  - Enters **low-power SLEEP** mode


# TI Fuel Gauge Operating Modes Specification

## Overview
The fuel gauge operates in six distinct power modes: OFF, SHUTDOWN, POR, INITIALIZATION, NORMAL, CONFIG UPDATE, and SLEEP.

## Operating Modes

### 1. OFF Mode
- **Power State**: REGIN pin = OFF, VCC pin = OFF
- **Status**: Complete power down
- **Entry**: System power removal
- **Exit**: Apply power to REGIN pin (> VREGIN min)

### 2. SHUTDOWN Mode
- **Power State**: REGIN pin > VREGIN min, VCC pin = OFF
- **Status**: LDO output disabled, all RAM-based volatile data lost
- **Entry**: Host sends SHUTDOWN_ENABLE subcommand followed by SHUTDOWN subcommand
- **Exit**: Raise GPOUT pin from logic low to logic high for ≥200 µs

### 3. Power-On Reset (POR) Mode
- **Function**: Copies ROM-based configuration defaults to RAM
- **Entry**: Automatic upon power-up or via RESET subcommand
- **Initialization**: Sets Flags()[ITPOR] status bit = 1
- **Status Clearing**: CONTROL_STATUS [QMAX_UP] and [RES_UP] bits cleared

### 4. INITIALIZATION Mode
- **Function**: Initialize algorithm and data, check for battery insertion
- **Gauging**: No gauging performed in this mode
- **Current**: ICC = Normal
- **Entry**: Automatic from POR when Flags()[BAT_DET] = 0
- **Exit**: When Flags()[BAT_DET] = 1 (battery detected)
- **Completion**: Indicated by CONTROL_STATUS [INITCOMP] bit

### 5. NORMAL Mode
- **Function**: Primary operating mode for fuel gauging
- **Measurements**: AverageCurrent(), Voltage(), and Temperature() taken once per second
- **Interface**: Data set is updated and state change decisions made
- **Power**: Highest power consumption mode
- **Current**: ICC = Normal
- **Optimization**: Impedance Track algorithm minimizes time in this mode

### 6. CONFIG UPDATE Mode
- **Function**: Update RAM-based configuration parameters
- **Entry**: Host sends Control() SET_CFGUPDATE subcommand
- **Status**: Operation indicated by Flags() [CFGUPMODE] status bit = 1
- **Gauging**: Fuel gauging suspended while host modifies configuration
- **Exit Methods**:
  - Host sends Control() SOFT_RESET subcommand
  - Automatic timeout after ~240 seconds (4 minutes)
- **Status Clearing**: Clears both Flags() [ITPOR] and [CFGUPMODE] bits

### 7. SLEEP Mode
- **Function**: Ultra-low power state with periodic wake-up
- **Power**: Very low-power idle state
- **Current**: ICC = Sleep
- **Wake Interval**: Automatically wakes every 20 seconds for measurements
- **Entry Conditions**:
  - OpConfig [SLEEP] = 1 (feature enabled) AND
  - AverageCurrent() < Sleep Current (default = 10 mA)
- **Exit Conditions**:
  - Host sets OpConfig [SLEEP] = 0 OR
  - AverageCurrent() > Sleep Current OR
  - Current detected above ±30 mA threshold
- **Calibration**: May perform ADC autocalibration before entering to minimize offset

## Key Features

### Power Management
- **Sleep Mode**: Periodic 20-second wake-up for data collection
- **Current Thresholds**: Default Sleep Current = 10 mA, wake-up threshold = ±30 mA
- **Power Optimization**: Impedance Track algorithm manages mode transitions for minimal power consumption

### Configuration Management
- **ROM Defaults**: Restored during POR mode
- **RAM Updates**: Performed in CONFIG UPDATE mode
- **Timeout Protection**: 240-second automatic exit from CONFIG UPDATE mode

### Status Monitoring
- **Flag Bits**: ITPOR, CFGUPMODE, BAT_DET, INITCOMP
- **Control Status**: QMAX_UP, RES_UP bits for learning status
- **Mode Indication**: Various status bits indicate current operating mode


*/

// BQ27426 I2C Address (7-bit address)
#define BQ27426_I2C_ADDRESS 0x55

// I2C Configuration
#define BQ27426_I2C_MASTER_NUM     I2C_NUM_0
#define BQ27426_I2C_MASTER_FREQ_HZ 100000
#define BQ27426_I2C_MASTER_TIMEOUT_MS 1000

// Standard Commands (from reference manual Table 5-1)
#define BQ27426_CONTROL             0x00  // Control()
#define BQ27426_TEMPERATURE         0x02  // Temperature()
#define BQ27426_VOLTAGE             0x04  // Voltage()
#define BQ27426_FLAGS               0x06  // Flags()
#define BQ27426_NOMINAL_CAPACITY    0x08  // NominalAvailableCapacity()
#define BQ27426_FULL_CAPACITY       0x0A  // FullAvailableCapacity()
#define BQ27426_REMAINING_CAPACITY  0x0C  // RemainingCapacity()
#define BQ27426_FULL_CHARGE_CAP     0x0E  // FullChargeCapacity()
#define BQ27426_AVERAGE_CURRENT     0x10  // AverageCurrent()
#define BQ27426_AVERAGE_POWER       0x18  // AveragePower()
#define BQ27426_STATE_OF_CHARGE     0x1C  // StateOfCharge()
#define BQ27426_INTERNAL_TEMP       0x1E  // InternalTemperature()
#define BQ27426_STATE_OF_HEALTH     0x20  // StateOfHealth()

// Control Subcommands (from reference manual Table 5-2)
#define BQ27426_CONTROL_STATUS      0x0000
#define BQ27426_DEVICE_TYPE         0x0001
#define BQ27426_FW_VERSION          0x0002
#define BQ27426_DM_CODE             0x0004
#define BQ27426_PREV_MACWRITE       0x0007
#define BQ27426_CHEM_ID             0x0008
#define BQ27426_BAT_INSERT          0x000C
#define BQ27426_BAT_REMOVE          0x000D
#define BQ27426_SET_HIBERNATE       0x0011
#define BQ27426_CLEAR_HIBERNATE     0x0012
#define BQ27426_SET_CFGUPDATE       0x0013
#define BQ27426_SHUTDOWN_ENABLE     0x001B
#define BQ27426_SHUTDOWN            0x001C
#define BQ27426_SEALED              0x0020
#define BQ27426_PULSE_SOC_INT       0x0023
#define BQ27426_CHEM_A              0x0030
#define BQ27426_CHEM_B              0x0031
#define BQ27426_CHEM_C              0x0032
#define BQ27426_RESET               0x0041
#define BQ27426_SOFT_RESET          0x0042
#define BQ27426_EXIT_CFGUPDATE      0x0043
#define BQ27426_EXIT_RESIM          0x0044

// Data Memory Commands
#define BQ27426_DATA_CLASS          0x3E
#define BQ27426_DATA_BLOCK          0x3F
#define BQ27426_BLOCK_DATA          0x40
#define BQ27426_BLOCK_DATA_CHECKSUM 0x60
#define BQ27426_BLOCK_DATA_CONTROL  0x61

// Error codes
typedef enum {
    TI_FUEL_GAUGE_OK = 0,
    TI_FUEL_GAUGE_ERR_I2C_COMM,
    TI_FUEL_GAUGE_ERR_DEVICE_NOT_FOUND,
    TI_FUEL_GAUGE_ERR_NOT_INITIALIZED
} ti_fuel_gauge_err_t;

// Chemical ID definitions
typedef enum {
    TI_CHEM_ID_4_35V = 0x3230,  // 4.35V (default)
    TI_CHEM_ID_4_2V  = 0x1202,  // 4.2V
    TI_CHEM_ID_4_4V  = 0x3142,  // 4.4V
    TI_CHEM_ID_UNKNOWN = 0xFFFF // Unknown/unsupported chemistry
} ti_fuel_gauge_chem_id_t;

// Control Status bit definitions
#define TI_CTRL_SHUTDOWNEN  0x8000  // Bit 15: Shutdown enabled
#define TI_CTRL_WDRESET     0x4000  // Bit 14: Watchdog reset occurred
#define TI_CTRL_SS          0x2000  // Bit 13: Sealed state
#define TI_CTRL_CALMODE     0x1000  // Bit 12: Calibration mode
#define TI_CTRL_CCA         0x0800  // Bit 11: Coulomb counter auto-calibration
#define TI_CTRL_BCA         0x0400  // Bit 10: Board calibration active
#define TI_CTRL_QMAX_UP     0x0200  // Bit 9: Qmax updated
#define TI_CTRL_RES_UP      0x0100  // Bit 8: Resistance updated
#define TI_CTRL_INITCOMP    0x0080  // Bit 7: Initialization complete
#define TI_CTRL_RSVD2       0x0040  // Bit 6: Reserved
#define TI_CTRL_RSVD1       0x0020  // Bit 5: Reserved
#define TI_CTRL_SLEEP       0x0010  // Bit 4: Sleep mode
#define TI_CTRL_LDMD        0x0008  // Bit 3: Low data memory density
#define TI_CTRL_RUP_DIS     0x0004  // Bit 2: Ra table updates disabled
#define TI_CTRL_VOK         0x0002  // Bit 1: Voltage OK
#define TI_CTRL_CHEM_CHANGE 0x0001  // Bit 0: Chemistry change

// Data Memory Access Constants
#define TI_STATE_SUBCLASS           0x52    // State subclass (82 decimal)
#define TI_DESIGN_CAPACITY_OFFSET   6       // Design capacity offset in State subclass
#define TI_CONFIG_UPDATE_WAIT_MS    1100    // Mandatory wait after SET_CFGUPDATE

// TI Fuel Gauge class
class TiFuelGauge {
public:
    TiFuelGauge();
    ~TiFuelGauge();
    
    // Device identification
    uint16_t read_device_type();
    ti_fuel_gauge_chem_id_t read_chemical_id();
    uint16_t read_control_status();
    
    // Battery measurements
    uint16_t read_voltage();           // Voltage in mV
    int16_t read_average_current();    // Average current in mA
    int16_t read_average_power();      // Average power in mW
    uint16_t read_state_of_charge();   // State of charge in %
    uint16_t read_full_capacity();     // Full available capacity in mAh
    
    // Basic I2C communication
    bool is_connected();
    
    // Status utilities
    const char* control_status_to_string(uint16_t status);
    void debug_print_status();
    
    // Control functions
    bool reset();
    
    // Configuration management
    bool enter_config_update_mode();
    bool exit_config_update_mode();
    bool is_config_update_mode();
    bool force_exit_config_mode();  // Emergency exit function
    
    // Parameter configuration
    bool set_design_capacity(uint16_t capacity_mah);
    bool set_chemistry_profile(ti_fuel_gauge_chem_id_t profile);
    uint16_t read_design_capacity_from_memory(); // Read design capacity from data memory

    bool enter_shutdown();
private:
    static const char* TAG;
    i2c_port_t i2c_port;
    bool initialized;
    
    // Low-level I2C functions
    esp_err_t i2c_master_read_register(uint8_t reg, uint8_t* data, size_t len);
    esp_err_t i2c_master_write_register(uint8_t reg, uint16_t value);
    
    // Register access functions
    uint16_t read_register(uint8_t reg);
    bool write_register(uint8_t reg, uint16_t value);
    bool write_control_command(uint16_t command);
    
    // Data memory access helpers
    uint8_t read_data_memory_byte(uint8_t address);
    bool write_data_memory_byte(uint8_t address, uint8_t value);
    uint8_t calculate_checksum_update(uint8_t old_checksum, uint8_t old_value, uint8_t new_value);
};

/*
 * USAGE EXAMPLE:
 * 
 * // Create TI Fuel Gauge instance
 * TiFuelGauge fuel_gauge;
 * 
 * // Check if device is connected
 * if (!fuel_gauge.is_connected()) {
 *     ESP_LOGE("MAIN", "TI Fuel Gauge not found on I2C bus");
 *     return;
 * }
 * 
 * // Read device type
 * uint16_t device_type = fuel_gauge.read_device_type();
 * if (device_type != 0xFFFF) {
 *     ESP_LOGI("MAIN", "TI Fuel Gauge device type: 0x%04X", device_type);
 * } else {
 *     ESP_LOGE("MAIN", "Failed to read device type");
 * }
 */