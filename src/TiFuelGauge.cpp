#include "TiFuelGauge.h"
#include <string.h>
#include <stdio.h>

const char* TiFuelGauge::TAG = "TiFuelGauge";

TiFuelGauge::TiFuelGauge() : i2c_port(BQ27426_I2C_MASTER_NUM), initialized(false) {
}

TiFuelGauge::~TiFuelGauge() {
}

bool TiFuelGauge::is_connected() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27426_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(BQ27426_I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    bool connected = (ret == ESP_OK);
    ESP_LOGI(TAG, "Connection test: %s (I2C address 0x%02X)", connected ? "SUCCESS" : "FAILED", BQ27426_I2C_ADDRESS);
    if (!connected) {
        ESP_LOGI(TAG, "I2C connection error: %s", esp_err_to_name(ret));
    }
    
    return connected;
}

esp_err_t TiFuelGauge::i2c_master_read_register(uint8_t reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27426_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    // Read data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27426_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(BQ27426_I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t TiFuelGauge::i2c_master_write_register(uint8_t reg, uint16_t value) {
    uint8_t data[3] = {reg, (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF)};
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27426_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 3, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(BQ27426_I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

uint16_t TiFuelGauge::read_register(uint8_t reg) {
    uint8_t data[2];
    esp_err_t ret = i2c_master_read_register(reg, data, 2);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(ret));
        return 0xFFFF;
    }
    
    // TI Fuel Gauge returns LSB first, then MSB (same as BQ27426)
    uint16_t value = (data[1] << 8) | data[0];
    return value;
}

bool TiFuelGauge::write_register(uint8_t reg, uint16_t value) {
    esp_err_t ret = i2c_master_write_register(reg, value);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
        return false;
    }
    return true;
}

bool TiFuelGauge::write_control_command(uint16_t command) {
    return write_register(BQ27426_CONTROL, command);
}

uint16_t TiFuelGauge::read_device_type() {
    write_control_command(BQ27426_DEVICE_TYPE);
    vTaskDelay(pdMS_TO_TICKS(2));
    return read_register(BQ27426_CONTROL);
}

ti_fuel_gauge_chem_id_t TiFuelGauge::read_chemical_id() {
    write_control_command(BQ27426_CHEM_ID);
    vTaskDelay(pdMS_TO_TICKS(2));
    uint16_t raw_value = read_register(BQ27426_CONTROL);
    
    // Check if it's a known chemistry ID
    switch (raw_value) {
        case TI_CHEM_ID_4_35V:
        case TI_CHEM_ID_4_2V:
        case TI_CHEM_ID_4_4V:
            return static_cast<ti_fuel_gauge_chem_id_t>(raw_value);
        default:
            ESP_LOGW(TAG, "Unknown chemical ID: 0x%04X", raw_value);
            return TI_CHEM_ID_UNKNOWN;
    }
}

uint16_t TiFuelGauge::read_control_status() {
    write_control_command(BQ27426_CONTROL_STATUS);
    vTaskDelay(pdMS_TO_TICKS(2));
    return read_register(BQ27426_CONTROL);
}

const char* TiFuelGauge::control_status_to_string(uint16_t status) {
    static char statusStr[512];
    statusStr[0] = '\0';
    
    // High byte status bits (15-8)
    if (status & TI_CTRL_SHUTDOWNEN) strcat(statusStr, "SHUTDOWN_EN ");
    if (status & TI_CTRL_WDRESET) strcat(statusStr, "WD_RESET ");
    if (status & TI_CTRL_SS) strcat(statusStr, "SEALED ");
    if (status & TI_CTRL_CALMODE) strcat(statusStr, "CAL_MODE ");
    if (status & TI_CTRL_CCA) strcat(statusStr, "CCA_ACTIVE ");
    if (status & TI_CTRL_BCA) strcat(statusStr, "BCA_ACTIVE ");
    if (status & TI_CTRL_QMAX_UP) strcat(statusStr, "QMAX_UPDATED ");
    if (status & TI_CTRL_RES_UP) strcat(statusStr, "RES_UPDATED ");
    
    // Low byte status bits (7-0)
    if (status & TI_CTRL_INITCOMP) strcat(statusStr, "INIT_COMPLETE ");
    if (status & TI_CTRL_SLEEP) strcat(statusStr, "SLEEP_MODE ");
    if (status & TI_CTRL_LDMD) strcat(statusStr, "LDMD ");
    if (status & TI_CTRL_RUP_DIS) strcat(statusStr, "RA_UPDATES_DIS ");
    if (status & TI_CTRL_VOK) strcat(statusStr, "VOLTAGE_OK ");
    if (status & TI_CTRL_CHEM_CHANGE) strcat(statusStr, "CHEM_CHANGED ");
    
    // Remove trailing space if any
    size_t len = strlen(statusStr);
    if (len > 0 && statusStr[len - 1] == ' ') {
        statusStr[len - 1] = '\0';
    }
    
    // If no flags are set, return a clear indication
    if (statusStr[0] == '\0') {
        strcpy(statusStr, "NO_FLAGS_SET");
    }
    
    return statusStr;
}

void TiFuelGauge::debug_print_status() {
    ESP_LOGI(TAG, "=== TI Fuel Gauge Debug Status ===");
    
    uint16_t device_type = read_device_type();
    ESP_LOGI(TAG, "Device Type: 0x%04X", device_type);
    
    ti_fuel_gauge_chem_id_t chem_id = read_chemical_id();
    ESP_LOGI(TAG, "Chemical ID: 0x%04X", chem_id);
    
    uint16_t control_status = read_control_status();
    ESP_LOGI(TAG, "Control Status: 0x%04X (%s)", control_status, control_status_to_string(control_status));
    
    uint16_t flags = read_register(BQ27426_FLAGS);
    ESP_LOGI(TAG, "FLAGS: 0x%04X", flags);
    ESP_LOGI(TAG, "CONFIG_UPDATE Mode: %s", is_config_update_mode() ? "YES" : "NO");
    
    ESP_LOGI(TAG, "=== End Debug Status ===");
}

uint16_t TiFuelGauge::read_voltage() {
    return read_register(BQ27426_VOLTAGE);
}

int16_t TiFuelGauge::read_average_current() {
    return (int16_t)read_register(BQ27426_AVERAGE_CURRENT);
}

int16_t TiFuelGauge::read_average_power() {
    return (int16_t)read_register(BQ27426_AVERAGE_POWER);
}

uint16_t TiFuelGauge::read_state_of_charge() {
    return read_register(BQ27426_STATE_OF_CHARGE);
}

uint16_t TiFuelGauge::read_full_capacity() {
    return read_register(BQ27426_FULL_CAPACITY);
}

bool TiFuelGauge::reset() {
    ESP_LOGI(TAG, "Performing fuel gauge reset...");
    if (!write_control_command(BQ27426_RESET)) {
        ESP_LOGE(TAG, "Failed to send reset command");
        return false;
    }
    
    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "Fuel gauge reset complete");
    return true;
}

bool TiFuelGauge::enter_config_update_mode() {
    ESP_LOGI(TAG, "Entering CONFIG_UPDATE mode...");
    
    if (!write_control_command(BQ27426_SET_CFGUPDATE)) {
        ESP_LOGE(TAG, "Failed to send SET_CFGUPDATE command");
        return false;
    }
    
    // Wait mandatory 1100ms for mode entry
    ESP_LOGI(TAG, "Waiting %d ms for CONFIG_UPDATE mode entry...", TI_CONFIG_UPDATE_WAIT_MS);
    vTaskDelay(pdMS_TO_TICKS(TI_CONFIG_UPDATE_WAIT_MS));
    
    // Verify we're in config update mode by checking FLAGS bit 4
    if (!is_config_update_mode()) {
        ESP_LOGE(TAG, "Failed to enter CONFIG_UPDATE mode");
        return false;
    }
    
    ESP_LOGI(TAG, "Successfully entered CONFIG_UPDATE mode");
    return true;
}

bool TiFuelGauge::exit_config_update_mode() {
    ESP_LOGI(TAG, "Exiting CONFIG_UPDATE mode...");
    
    if (!write_control_command(BQ27426_SOFT_RESET)) {
        ESP_LOGE(TAG, "Failed to send SOFT_RESET command");
        return false;
    }
    
    // Wait for soft reset to complete and poll for exit
    for (int i = 0; i < 50; i++) {  // Wait up to 5 seconds
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!is_config_update_mode()) {
            ESP_LOGI(TAG, "Successfully exited CONFIG_UPDATE mode after %d ms", (i + 1) * 100);
            return true;
        }
    }
    
    ESP_LOGE(TAG, "Failed to exit CONFIG_UPDATE mode after 5 seconds");
    debug_print_status();
    return false;
}

bool TiFuelGauge::is_config_update_mode() {
    // Read FLAGS register to check bit 4 (CFGUPMODE)
    uint16_t flags = read_register(BQ27426_FLAGS);
    if (flags == 0xFFFF) {
        ESP_LOGE(TAG, "Failed to read FLAGS register");
        return false;
    }
    
    // Check bit 4 (0x0010) for CONFIG_UPDATE mode
    return (flags & 0x0010) != 0;
}

bool TiFuelGauge::force_exit_config_mode() {
    ESP_LOGW(TAG, "Force exiting CONFIG_UPDATE mode (emergency procedure)...");
    
    // Try multiple methods to exit config mode
    
    // Method 1: Send SOFT_RESET multiple times
    for (int i = 0; i < 3; i++) {
        write_control_command(BQ27426_SOFT_RESET);
        vTaskDelay(pdMS_TO_TICKS(200));
        if (!is_config_update_mode()) {
            ESP_LOGI(TAG, "Force exit successful with SOFT_RESET attempt %d", i + 1);
            return true;
        }
    }
    
    // Method 2: Try RESET command
    ESP_LOGW(TAG, "SOFT_RESET failed, trying full RESET...");
    if (write_control_command(BQ27426_RESET)) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (!is_config_update_mode()) {
            ESP_LOGI(TAG, "Force exit successful with RESET");
            return true;
        }
    }
    
    ESP_LOGE(TAG, "Force exit failed - device may need power cycle");
    debug_print_status();
    return false;
}

uint8_t TiFuelGauge::read_data_memory_byte(uint8_t address) {
    uint8_t data;
    esp_err_t ret = i2c_master_read_register(address, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data memory byte at 0x%02X: %s", address, esp_err_to_name(ret));
        return 0xFF;
    }
    return data;
}

bool TiFuelGauge::write_data_memory_byte(uint8_t address, uint8_t value) {
    // Add small delay before write to ensure device is ready
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // For data memory writes, we need exactly 2 bytes: register + value
    // Following specs format: 0xAA register value
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27426_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(BQ27426_I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data memory byte 0x%02X to 0x%02X: %s", value, address, esp_err_to_name(ret));
        
        // Check if device is still connected
        if (!is_connected()) {
            ESP_LOGE(TAG, "Device connection lost during data memory write");
        }
        return false;
    }
    
    ESP_LOGD(TAG, "Successfully wrote 0x%02X to address 0x%02X", value, address);
    return true;
}

uint8_t TiFuelGauge::calculate_checksum_update(uint8_t old_checksum, uint8_t old_value, uint8_t new_value) {
    // Formula from specs: temp = (255 - OLD_Csum - OLD_Value + NEW_Value) % 256
    //                     NEW_Csum = (255 - temp) % 256
    uint16_t temp = (255 - old_checksum - old_value + new_value) % 256;
    uint8_t new_checksum = (255 - temp) % 256;
    return new_checksum;
}

bool TiFuelGauge::set_design_capacity(uint16_t capacity_mah) {
    ESP_LOGI(TAG, "Setting design capacity to %d mAh", capacity_mah);
    
    // Validate input range
    if (capacity_mah > 8000) {
        ESP_LOGE(TAG, "Design capacity %d mAh exceeds maximum of 8000 mAh", capacity_mah);
        return false;
    }
    
    // Enter CONFIG_UPDATE mode
    if (!enter_config_update_mode()) {
        ESP_LOGE(TAG, "Failed to enter CONFIG_UPDATE mode");
        return false;
    }
    
    // Following the exact sequence from TiFuelGaugeSpecsUpdate.md
    bool success = false;
    
    // Declare all variables before any goto statements
    uint8_t old_msb, old_lsb, new_msb, new_lsb;
    uint8_t calculated_checksum;
    uint16_t old_capacity, sum;
    uint8_t block_data[32];
    uint8_t verify_msb, verify_lsb, verify_checksum;
    uint16_t verify_capacity;
    
    // 1. Enable block data control
    if (!write_data_memory_byte(BQ27426_BLOCK_DATA_CONTROL, 0x00)) {
        ESP_LOGE(TAG, "Failed to enable block data control");
        goto exit_config;
    }
    
    // 2. Access State subclass (0x52)
    if (!write_data_memory_byte(BQ27426_DATA_CLASS, TI_STATE_SUBCLASS)) {
        ESP_LOGE(TAG, "Failed to set data class to State subclass");
        goto exit_config;
    }
    
    // 3. Set block offset 0 (data 0-31)
    if (!write_data_memory_byte(BQ27426_DATA_BLOCK, 0x00)) {
        ESP_LOGE(TAG, "Failed to set block offset");
        goto exit_config;
    }
    
    // 4. Skip reading old checksum (not needed for block-based calculation)
    
    // 5. Read current Design Capacity (MSB at 0x4A, LSB at 0x4B)
    old_msb = read_data_memory_byte(BQ27426_BLOCK_DATA + TI_DESIGN_CAPACITY_OFFSET);
    old_lsb = read_data_memory_byte(BQ27426_BLOCK_DATA + TI_DESIGN_CAPACITY_OFFSET + 1);
    old_capacity = (old_msb << 8) | old_lsb;
    
    ESP_LOGI(TAG, "Current design capacity: %d mAh, changing to %d mAh", old_capacity, capacity_mah);
    
    // 6. Calculate new values (big-endian format)
    new_msb = (capacity_mah >> 8) & 0xFF;
    new_lsb = capacity_mah & 0xFF;
    
    // 7. Write new Design Capacity MSB
    if (!write_data_memory_byte(BQ27426_BLOCK_DATA + TI_DESIGN_CAPACITY_OFFSET, new_msb)) {
        ESP_LOGE(TAG, "Failed to write new Design Capacity MSB");
        goto exit_config;
    }
    
    // 8. Write new Design Capacity LSB
    if (!write_data_memory_byte(BQ27426_BLOCK_DATA + TI_DESIGN_CAPACITY_OFFSET + 1, new_lsb)) {
        ESP_LOGE(TAG, "Failed to write new Design Capacity LSB");
        goto exit_config;
    }
    
    // 9. Calculate checksum: read entire block and compute
    for (int i = 0; i < 32; i++) {
        block_data[i] = read_data_memory_byte(BQ27426_BLOCK_DATA + i);
    }
    
    sum = 0;
    for (int i = 0; i < 32; i++) {
        sum += block_data[i];
    }
    calculated_checksum = (255 - (sum % 256)) % 256;
    
    // 10. Write calculated checksum
    if (!write_data_memory_byte(BQ27426_BLOCK_DATA_CHECKSUM, calculated_checksum)) {
        ESP_LOGE(TAG, "Failed to write checksum to address 0x%02X", BQ27426_BLOCK_DATA_CHECKSUM);
        goto exit_config;
    }
    
    // 11. Verify the data was written correctly
    verify_msb = read_data_memory_byte(BQ27426_BLOCK_DATA + TI_DESIGN_CAPACITY_OFFSET);
    verify_lsb = read_data_memory_byte(BQ27426_BLOCK_DATA + TI_DESIGN_CAPACITY_OFFSET + 1);
    verify_capacity = (verify_msb << 8) | verify_lsb;
    verify_checksum = read_data_memory_byte(BQ27426_BLOCK_DATA_CHECKSUM);
    
    if (verify_capacity != capacity_mah) {
        ESP_LOGE(TAG, "Design capacity verification failed: wrote %d, read %d", capacity_mah, verify_capacity);
        goto exit_config;
    }
    
    if (verify_checksum != calculated_checksum) {
        ESP_LOGE(TAG, "Checksum verification failed: wrote 0x%02X, read 0x%02X", calculated_checksum, verify_checksum);
        goto exit_config;
    }
    
    ESP_LOGI(TAG, "Design capacity updated: %d → %d mAh (verified)", old_capacity, capacity_mah);
    success = true;
    
exit_config:
    // Exit CONFIG_UPDATE mode
    if (!exit_config_update_mode()) {
        ESP_LOGW(TAG, "Warning: Failed to cleanly exit CONFIG_UPDATE mode");
        success = false;
    }
    
    if (success) {
        ESP_LOGI(TAG, "Design capacity configuration complete. Fuel gauge will recalibrate.");
        
        // Final verification: check if the fuel gauge is using the new design capacity
        // by reading functional registers instead of data memory
        ESP_LOGI(TAG, "Performing final verification via functional registers...");
        vTaskDelay(pdMS_TO_TICKS(100)); // Brief delay for parameters to take effect
        
        uint16_t full_capacity = read_full_capacity();
        ESP_LOGI(TAG, "Full Available Capacity: %d mAh", full_capacity);
        
        // The full capacity should be close to the design capacity if the update worked
        // Allow some tolerance since the fuel gauge may adjust based on battery condition
        if (full_capacity > (capacity_mah * 8 / 10) && full_capacity < (capacity_mah * 12 / 10)) {
            ESP_LOGI(TAG, "Final verification SUCCESS: Fuel gauge is using new design capacity");
            ESP_LOGI(TAG, "Expected ~%d mAh, measured %d mAh (within acceptable range)", capacity_mah, full_capacity);
        } else {
            ESP_LOGW(TAG, "Final verification: Full capacity %d mAh doesn't match expected %d mAh", full_capacity, capacity_mah);
            ESP_LOGW(TAG, "This may be normal if battery condition differs from design parameters");
        }
    } else {
        ESP_LOGE(TAG, "Design capacity update failed");
    }
    
    return success;
}

uint16_t TiFuelGauge::read_design_capacity_from_memory() {
    ESP_LOGI(TAG, "Reading design capacity from data memory...");
    
    if (!enter_config_update_mode()) {
        ESP_LOGE(TAG, "Failed to enter CONFIG_UPDATE mode for reading design capacity");
        return 0xFFFF;
    }
    
    uint16_t capacity = 0xFFFF;
    
    // Access the design capacity data memory location
    if (write_data_memory_byte(BQ27426_BLOCK_DATA_CONTROL, 0x00) &&
        write_data_memory_byte(BQ27426_DATA_CLASS, TI_STATE_SUBCLASS) &&
        write_data_memory_byte(BQ27426_DATA_BLOCK, 0x00)) {
        
        uint8_t msb = read_data_memory_byte(BQ27426_BLOCK_DATA + TI_DESIGN_CAPACITY_OFFSET);
        uint8_t lsb = read_data_memory_byte(BQ27426_BLOCK_DATA + TI_DESIGN_CAPACITY_OFFSET + 1);
        capacity = (msb << 8) | lsb;
        
        ESP_LOGI(TAG, "Design capacity from data memory: %d mAh", capacity);
    } else {
        ESP_LOGE(TAG, "Failed to access design capacity data memory");
    }
    
    exit_config_update_mode();
    return capacity;
}

bool TiFuelGauge::set_chemistry_profile(ti_fuel_gauge_chem_id_t profile) {
    ESP_LOGI(TAG, "Setting chemistry profile to 0x%04X", profile);
    
    // Map enum to control command
    uint16_t command;
    const char* profile_name;
    
    switch (profile) {
        case TI_CHEM_ID_4_35V:
            command = BQ27426_CHEM_A;
            profile_name = "4.35V (Profile A)";
            break;
        case TI_CHEM_ID_4_2V:
            command = BQ27426_CHEM_B;
            profile_name = "4.2V (Profile B)";
            break;
        case TI_CHEM_ID_4_4V:
            command = BQ27426_CHEM_C;
            profile_name = "4.4V (Profile C)";
            break;
        default:
            ESP_LOGE(TAG, "Invalid chemistry profile: 0x%04X", profile);
            return false;
    }
    
    ESP_LOGI(TAG, "Changing to chemistry profile: %s", profile_name);
    
    // Read current chemistry ID for comparison
    ti_fuel_gauge_chem_id_t current_chem = read_chemical_id();
    if (current_chem == profile) {
        ESP_LOGI(TAG, "Chemistry profile already set to 0x%04X", profile);
        return true;
    }
    
    // Enter CONFIG_UPDATE mode
    if (!enter_config_update_mode()) {
        ESP_LOGE(TAG, "Failed to enter CONFIG_UPDATE mode");
        return false;
    }
    
    // Send chemistry change command
    bool success = false;
    if (!write_control_command(command)) {
        ESP_LOGE(TAG, "Failed to send chemistry change command 0x%04X", command);
        goto exit_config;
    }
    
    ESP_LOGI(TAG, "Chemistry change command sent successfully");
    success = true;
    
exit_config:
    // Exit CONFIG_UPDATE mode
    if (!exit_config_update_mode()) {
        ESP_LOGW(TAG, "Warning: Failed to cleanly exit CONFIG_UPDATE mode");
        success = false;
    }
    
    if (success) {
        // Wait for chemistry change to take effect
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Verify the change
        ti_fuel_gauge_chem_id_t new_chem = read_chemical_id();
        if (new_chem == profile) {
            ESP_LOGI(TAG, "Chemistry profile successfully changed to %s (0x%04X)", profile_name, profile);
            
            // Check for CHEM_CHANGE flag in control status
            uint16_t status = read_control_status();
            if (status & TI_CTRL_CHEM_CHANGE) {
                ESP_LOGI(TAG, "Chemistry change flag detected in control status");
            }
        } else {
            ESP_LOGE(TAG, "Chemistry profile verification failed. Expected 0x%04X, got 0x%04X", profile, new_chem);
            success = false;
        }
    }
    
    if (success) {
        ESP_LOGI(TAG, "Chemistry profile configuration complete. SOC will be valid after 2 seconds.");
    } else {
        ESP_LOGE(TAG, "Chemistry profile update failed");
    }
    
    return success;
}

bool TiFuelGauge::enter_shutdown() {
    ESP_LOGI(TAG, "Requesting shutdown mode...");

    // Step 1: Send SHUTDOWN_ENABLE (0x001B)
    if (!write_control_command(0x001B)) {
        ESP_LOGE(TAG, "Failed to send SHUTDOWN_ENABLE command");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    // Step 2: Send SHUTDOWN (0x001C)
    if (!write_control_command(0x001C)) {
        ESP_LOGE(TAG, "Failed to send SHUTDOWN command");
        return false;
    }

    ESP_LOGI(TAG, "Shutdown command sent, waiting for gauge to power down...");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Step 3: Verify by probing I2C
    if (is_connected()) {
        ESP_LOGW(TAG, "Gauge still responds on I2C – shutdown may have failed.");
        return false;
    } else {
        ESP_LOGI(TAG, "Gauge no longer responds: confirmed in SHUTDOWN mode.");
        return true;
    }
}