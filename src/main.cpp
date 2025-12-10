#include "main.h"

void setup() {
    #ifndef DISABLE_USB_SERIAL
    Serial.begin(115200);
    delay(100);
    #endif
    writeSerial("=== FIRMWARE INFO ===");
    writeSerial("Firmware Version: " + String(getFirmwareMajor()) + "." + String(getFirmwareMinor()));
    const char* shaCStr = SHA_STRING;
    String shaStr = String(shaCStr);
    if (shaStr.length() >= 2 && shaStr.charAt(0) == '"' && shaStr.charAt(shaStr.length() - 1) == '"') {
        shaStr = shaStr.substring(1, shaStr.length() - 1);
    }
    if (shaStr.length() > 0 && shaStr != "\"\"" && shaStr != "") {
        writeSerial("Git SHA: " + shaStr);
    } else {
        writeSerial("Git SHA: (not set)");
    }
    writeSerial("Starting setup...");
    full_config_init();
    initio();
    ble_init();
    writeSerial("BLE advertising started - waiting for connections...");
    initDisplay();
    writeSerial("Display initialized");
    writeSerial("=== Setup completed successfully ===");
    updatemsdata();
}

void loop() {
    #ifdef TARGET_ESP32
    if (commandQueueTail != commandQueueHead) {
        writeSerial("ESP32: Processing queued command (" + String(commandQueue[commandQueueTail].len) + " bytes)");
        imageDataWritten(NULL, NULL, commandQueue[commandQueueTail].data, commandQueue[commandQueueTail].len);
        commandQueue[commandQueueTail].pending = false;
        commandQueueTail = (commandQueueTail + 1) % COMMAND_QUEUE_SIZE;
        writeSerial("Command processed");
    }
    if (responseQueueTail != responseQueueHead && pTxCharacteristic && pServer && pServer->getConnectedCount() > 0) {
        writeSerial("ESP32: Sending queued response (" + String(responseQueue[responseQueueTail].len) + " bytes)");
        pTxCharacteristic->setValue(responseQueue[responseQueueTail].data, responseQueue[responseQueueTail].len);
        pTxCharacteristic->notify();
        responseQueue[responseQueueTail].pending = false;
        responseQueueTail = (responseQueueTail + 1) % RESPONSE_QUEUE_SIZE;
        writeSerial("Response sent successfully");
        delay(20); // Brief delay to let BLE stack process
    }
    bool bleActive = (commandQueueTail != commandQueueHead) || 
                     (responseQueueTail != responseQueueHead) ||
                     (pServer && pServer->getConnectedCount() > 0);
    
    if (bleActive) {
        delay(10);
    } else {
        if(globalConfig.power_option.sleep_timeout_ms > 0){
            delay(globalConfig.power_option.sleep_timeout_ms);
            updatemsdata();
        }
        else{
            delay(2000);
        }
    }
    #else
    if(globalConfig.power_option.sleep_timeout_ms > 0){
        uint32_t remainingDelay = globalConfig.power_option.sleep_timeout_ms;
        const uint32_t CHECK_INTERVAL_MS = 100;
        while(remainingDelay > 0){
            uint32_t chunkDelay = (remainingDelay > CHECK_INTERVAL_MS) ? CHECK_INTERVAL_MS : remainingDelay;
            delay(chunkDelay);
            remainingDelay -= chunkDelay;
            //sd_app_evt_wait();
        }
        updatemsdata();
    }
    else{
        delay(500);
    }
    #endif
    writeSerial("Loop end: " + String(millis() / 100));
}

void initio(){
    if(globalConfig.led_count > 0){
    pinMode(globalConfig.leds[0].led_1_r, OUTPUT);
    pinMode(globalConfig.leds[0].led_2_g, OUTPUT);
    pinMode(globalConfig.leds[0].led_3_b, OUTPUT);
    delay(100);
    digitalWrite(globalConfig.leds[0].led_1_r, LOW);
    digitalWrite(globalConfig.leds[0].led_2_g, LOW);
    digitalWrite(globalConfig.leds[0].led_3_b, LOW);
    delay(100);
    digitalWrite(globalConfig.leds[0].led_1_r, HIGH);
    digitalWrite(globalConfig.leds[0].led_2_g, HIGH);
    digitalWrite(globalConfig.leds[0].led_3_b, HIGH);
    }
    
    if(globalConfig.system_config.pwr_pin != 0xFF){
    pinMode(globalConfig.system_config.pwr_pin, OUTPUT);
    digitalWrite(globalConfig.system_config.pwr_pin, LOW);
    }
    else{
        writeSerial("Power pin not set");
    }
    initDataBuses();
    initSensors();
}

uint8_t getFirmwareMajor(){
    String version = String(BUILD_VERSION);
    int dotIndex = version.indexOf('.');
    if (dotIndex > 0) {
        return version.substring(0, dotIndex).toInt();
    }
    return 0;
}

uint8_t getFirmwareMinor(){
    String version = String(BUILD_VERSION);
    int dotIndex = version.indexOf('.');
    if (dotIndex > 0 && dotIndex < (int)(version.length() - 1)) {
        return version.substring(dotIndex + 1).toInt();
    }
    return 0;
}

void initDataBuses(){
    writeSerial("=== Initializing Data Buses ===");
    if(globalConfig.data_bus_count == 0){
        writeSerial("No data buses configured");
        return;
    }
    for(uint8_t i = 0; i < globalConfig.data_bus_count; i++){
        struct DataBus* bus = &globalConfig.data_buses[i];
        if(bus->bus_type == 0x01){ // I2C bus
            writeSerial("Initializing I2C bus " + String(i) + " (instance " + String(bus->instance_number) + ")");
            if(bus->pin_1 == 0xFF || bus->pin_2 == 0xFF){
                writeSerial("ERROR: Invalid I2C pins for bus " + String(i) + " (SCL=" + String(bus->pin_1) + ", SDA=" + String(bus->pin_2) + ")");
                continue;
            }
            uint32_t busSpeed = (bus->bus_speed_hz > 0) ? bus->bus_speed_hz : 100000;
            #ifdef TARGET_ESP32
            pinMode(bus->pin_1, INPUT);
            pinMode(bus->pin_2, INPUT);
            if(bus->pullups & 0x01){
                pinMode(bus->pin_1, INPUT_PULLUP);
            }
            if(bus->pullups & 0x02){
                pinMode(bus->pin_2, INPUT_PULLUP);
            }
            if(bus->pulldowns & 0x01){
                pinMode(bus->pin_1, INPUT_PULLDOWN);
            }
            if(bus->pulldowns & 0x02){
                pinMode(bus->pin_2, INPUT_PULLDOWN);
            }
            #endif
            #ifdef TARGET_NRF
            pinMode(bus->pin_1, INPUT);
            pinMode(bus->pin_2, INPUT);
            if(bus->pullups & 0x01){
                pinMode(bus->pin_1, INPUT_PULLUP);
            }
            if(bus->pullups & 0x02){
                pinMode(bus->pin_2, INPUT_PULLUP);
            }
            #endif
            if(i == 0){
                #ifdef TARGET_ESP32
                Wire.begin(bus->pin_2, bus->pin_1); // SDA, SCL
                Wire.setClock(busSpeed);
                #endif
                #ifdef TARGET_NRF
                Wire.begin(); // Uses default I2C pins
                Wire.setClock(busSpeed);
                writeSerial("NOTE: nRF52840 using default I2C pins (config pins: SCL=" + String(bus->pin_1) + ", SDA=" + String(bus->pin_2) + ")");
                #endif
                writeSerial("I2C bus " + String(i) + " initialized: SCL=pin" + String(bus->pin_1) + ", SDA=pin" + String(bus->pin_2) + ", Speed=" + String(busSpeed) + "Hz");
                scanI2CDevices();
            } else {
                writeSerial("WARNING: I2C bus " + String(i) + " configured but not initialized (only first bus supported)");
                writeSerial("  SCL=pin" + String(bus->pin_1) + ", SDA=pin" + String(bus->pin_2) + ", Speed=" + String(busSpeed) + "Hz");
            }
        }
        else if(bus->bus_type == 0x02){
            writeSerial("SPI bus " + String(i) + " detected (not yet implemented)");
            writeSerial("  Instance: " + String(bus->instance_number));
        }
        else{
            writeSerial("WARNING: Unknown bus type 0x" + String(bus->bus_type, HEX) + " for bus " + String(i));
        }
    }
    writeSerial("=== Data Bus Initialization Complete ===");
}

void scanI2CDevices(){
    writeSerial("=== Scanning I2C Bus for Devices ===");
    uint8_t deviceCount = 0;
    uint8_t foundDevices[128];
    for(uint8_t address = 0x08; address < 0x78; address++){
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        if(error == 0){
            foundDevices[deviceCount] = address;
            deviceCount++;
            writeSerial("I2C device found at address 0x" + String(address, HEX) + " (" + String(address) + ")");
        }
        else if(error == 4){
            writeSerial("ERROR: Unknown error at address 0x" + String(address, HEX));
        }
    }
    if(deviceCount == 0){
        writeSerial("No I2C devices found on bus");
    } else {
        writeSerial("Found " + String(deviceCount) + " I2C device(s)");
        writeSerial("Device addresses: ");
        String addrList = "";
        for(uint8_t i = 0; i < deviceCount; i++){
            if(i > 0) addrList += ", ";
            addrList += "0x" + String(foundDevices[i], HEX);
        }
        writeSerial(addrList);
    }
    writeSerial("=== I2C Scan Complete ===");
}

void initSensors(){
    writeSerial("=== Initializing Sensors ===");
    if(globalConfig.sensor_count == 0){
        writeSerial("No sensors configured");
        return;
    }
    for(uint8_t i = 0; i < globalConfig.sensor_count; i++){
        struct SensorData* sensor = &globalConfig.sensors[i];
        writeSerial("Initializing sensor " + String(i) + " (instance " + String(sensor->instance_number) + ")");
        writeSerial("  Type: 0x" + String(sensor->sensor_type, HEX));
        writeSerial("  Bus ID: " + String(sensor->bus_id));
        if(sensor->sensor_type == 0x0003){ // AXP2101 PMIC
            writeSerial("  Detected AXP2101 PMIC sensor");
            initAXP2101(sensor->bus_id);
            delay(100);
            readAXP2101Data();
        }
        else if(sensor->sensor_type == 0x0001){ // Temperature sensor
            writeSerial("  Temperature sensor (initialization not implemented)");
        }
        else if(sensor->sensor_type == 0x0002){ // Humidity sensor
            writeSerial("  Humidity sensor (initialization not implemented)");
        }
        else{
            writeSerial("  Unknown sensor type 0x" + String(sensor->sensor_type, HEX));
        }
    }
    writeSerial("=== Sensor Initialization Complete ===");
}

void initAXP2101(uint8_t busId){
    writeSerial("=== Initializing AXP2101 PMIC ===");
    if(busId >= globalConfig.data_bus_count){
        writeSerial("ERROR: Invalid bus ID " + String(busId) + " (only " + String(globalConfig.data_bus_count) + " buses configured)");
        return;
    }
    struct DataBus* bus = &globalConfig.data_buses[busId];
    if(bus->bus_type != 0x01){
        writeSerial("ERROR: Bus " + String(busId) + " is not an I2C bus");
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX) + " (error: " + String(error) + ")");
        return;
    }
    writeSerial("AXP2101 detected at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX));
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t status = Wire.read();
            writeSerial("Power status: 0x" + String(status, HEX));
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_VOL0_CTRL);
    Wire.write(0x12); // 18 decimal = 0x12, gives 3.3V (1500 + 18*100 = 3300)
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DCDC1 voltage set to 3.3V");
    } else {
        writeSerial("ERROR: Failed to set DCDC1 voltage");
    }
    delay(10); // Small delay after setting voltage
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    uint8_t dcEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            dcEnable = Wire.read();
        }
    }
    dcEnable |= 0x01;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    Wire.write(dcEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DCDC1 enabled (3.3V)");
    } else {
        writeSerial("ERROR: Failed to enable DCDC1");
    }
    delay(10);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    uint8_t aldoEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldoEnable = Wire.read();
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL2_CTRL);
    error = Wire.endTransmission();
    uint8_t aldo3VolReg = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldo3VolReg = Wire.read();
        }
    }
    aldo3VolReg = (aldo3VolReg & 0xE0) | 0x1C;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL2_CTRL);
    Wire.write(aldo3VolReg);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO3 voltage set to 3.3V");
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL3_CTRL);
    error = Wire.endTransmission();
    uint8_t aldo4VolReg = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldo4VolReg = Wire.read();
        }
    }
    aldo4VolReg = (aldo4VolReg & 0xE0) | 0x1C; // Preserve upper bits, set voltage
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL3_CTRL);
    Wire.write(aldo4VolReg);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO4 voltage set to 3.3V");
    }
    aldoEnable |= 0x0C; // Set bits 2 and 3 for ALDO3 and ALDO4
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    Wire.write(aldoEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO3 and ALDO4 enabled (3.3V)");
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t wakeupCtl = Wire.read();
            writeSerial("Wakeup control: 0x" + String(wakeupCtl, HEX));
            if(wakeupCtl & 0x01){
                writeSerial("Wakeup already enabled");
            } else {
                Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
                Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
                Wire.write(wakeupCtl | 0x01); // Set bit 0
                error = Wire.endTransmission();
                if(error == 0){
                    writeSerial("Wakeup enabled");
                }
            }
        }
    }
    writeSerial("=== AXP2101 PMIC Initialization Complete ===");
}

void readAXP2101Data(){
    writeSerial("=== Reading AXP2101 PMIC Data ===");
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX));
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_CHANNEL_CTRL);
    Wire.write(0xFF); // Enable all ADC channels
    error = Wire.endTransmission();
    delay(10); // Wait for ADC to stabilize
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t status1 = Wire.read();
            uint8_t status2 = Wire.read();
            writeSerial("Power Status 1: 0x" + String(status1, HEX));
            writeSerial("Power Status 2: 0x" + String(status2, HEX));
            bool batteryPresent = (status1 & 0x20) != 0;
            bool charging = (status1 & 0x04) != 0;
            bool vbusPresent = (status1 & 0x08) != 0;
            writeSerial("Battery Present: " + String(batteryPresent ? "Yes" : "No"));
            writeSerial("Charging: " + String(charging ? "Yes" : "No"));
            writeSerial("VBUS Present: " + String(vbusPresent ? "Yes" : "No"));
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_PWRON_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t pwronStatus = Wire.read();
            writeSerial("Power On Status: 0x" + String(pwronStatus, HEX));
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_BAT_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t batVolH = Wire.read();
            uint8_t batVolL = Wire.read();
            // Battery voltage = (H << 4 | L) * 0.5mV
            uint16_t batVolRaw = ((uint16_t)batVolH << 4) | (batVolL & 0x0F);
            float batVoltage = batVolRaw * 0.5; // in mV
            writeSerial("Battery Voltage: " + String(batVoltage, 1) + " mV (" + String(batVoltage / 1000.0, 2) + " V)");
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_VBUS_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t vbusVolH = Wire.read();
            uint8_t vbusVolL = Wire.read();
            // VBUS voltage = (H << 4 | L) * 1.7mV
            uint16_t vbusVolRaw = ((uint16_t)vbusVolH << 4) | (vbusVolL & 0x0F);
            float vbusVoltage = vbusVolRaw * 1.7; // in mV
            writeSerial("VBUS Voltage: " + String(vbusVoltage, 1) + " mV (" + String(vbusVoltage / 1000.0, 2) + " V)");
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_SYS_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t sysVolH = Wire.read();
            uint8_t sysVolL = Wire.read();
            // System voltage = (H << 4 | L) * 1.4mV
            uint16_t sysVolRaw = ((uint16_t)sysVolH << 4) | (sysVolL & 0x0F);
            float sysVoltage = sysVolRaw * 1.4; // in mV
            writeSerial("System Voltage: " + String(sysVoltage, 1) + " mV (" + String(sysVoltage / 1000.0, 2) + " V)");
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_BAT_PERCENT_DATA);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t batPercent = Wire.read();
            if(batPercent <= 100){
                writeSerial("Battery Percentage: " + String(batPercent) + "%");
            } else {
                writeSerial("Battery Percentage: Not available (fuel gauge may be disabled)");
            }
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t dcEnable = Wire.read();
            writeSerial("DC Enable Status: 0x" + String(dcEnable, HEX));
            writeSerial("  DCDC1: " + String((dcEnable & 0x01) ? "ON" : "OFF"));
            writeSerial("  DCDC2: " + String((dcEnable & 0x02) ? "ON" : "OFF"));
            writeSerial("  DCDC3: " + String((dcEnable & 0x04) ? "ON" : "OFF"));
            writeSerial("  DCDC4: " + String((dcEnable & 0x08) ? "ON" : "OFF"));
            writeSerial("  DCDC5: " + String((dcEnable & 0x10) ? "ON" : "OFF"));
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t aldoEnable = Wire.read();
            writeSerial("ALDO Enable Status: 0x" + String(aldoEnable, HEX));
            writeSerial("  ALDO1: " + String((aldoEnable & 0x01) ? "ON" : "OFF"));
            writeSerial("  ALDO2: " + String((aldoEnable & 0x02) ? "ON" : "OFF"));
            writeSerial("  ALDO3: " + String((aldoEnable & 0x04) ? "ON" : "OFF"));
            writeSerial("  ALDO4: " + String((aldoEnable & 0x08) ? "ON" : "OFF"));
        }
    }
    
    writeSerial("=== AXP2101 Data Read Complete ===");
}

void updatemsdata(){
    float batteryVoltage = readBatteryVoltage();
    float chipTemperature = readChipTemperature();
    writeSerial("Battery voltage: " + String(batteryVoltage) + "V");
    writeSerial("Chip temperature: " + String(chipTemperature) + "C");
    uint8_t chiptemp8 = (uint8_t)chipTemperature;
    uint16_t batteryVoltageMv = (uint16_t)(batteryVoltage * 1000);
    uint8_t batterymvvoltage8_high = (uint8_t)(batteryVoltageMv >> 8);
    uint8_t batterymvvoltage8_low = (uint8_t)(batteryVoltageMv & 0xFF);
uint8_t msd_payload[13];
uint16_t msd_cid = 0x2446;
memset(msd_payload, 0, sizeof(msd_payload));
memcpy(msd_payload, (uint8_t*)&msd_cid, sizeof(msd_cid));
msd_payload[2] = 0x02;
msd_payload[3] = 0x36;
msd_payload[4] = 0x00;
msd_payload[5] = 0x6C;
msd_payload[6] = 0x00;
msd_payload[7] = 0xC3;
msd_payload[8] = 0x01;
msd_payload[9] = batterymvvoltage8_low;
msd_payload[10] = batterymvvoltage8_high;
msd_payload[11] = chiptemp8;
msd_payload[12] = mloopcounter;
#ifdef TARGET_NRF
Bluefruit.Advertising.clearData();
Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
Bluefruit.Advertising.addName();
Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, msd_payload, sizeof(msd_payload));
Bluefruit.Advertising.setInterval(32, 400);
Bluefruit.Advertising.setFastTimeout(1);
Bluefruit.Advertising.stop();
Bluefruit.Advertising.start(0);
#endif
#ifdef TARGET_ESP32
if (advertisementData != nullptr) {
        String manufacturerDataStr;
        manufacturerDataStr.reserve(13);
        for (int i = 0; i < 13; i++) {
            manufacturerDataStr += (char)msd_payload[i];
        }
        advertisementData->setManufacturerData(manufacturerDataStr);
        BLEAdvertising *pAdvertising = nullptr;
        if (pServer != nullptr) {
            pAdvertising = pServer->getAdvertising();
        }
        if (pAdvertising == nullptr) {
            pAdvertising = BLEDevice::getAdvertising();
        }
        if (pAdvertising != nullptr) {
            pAdvertising->stop();
            BLEUUID serviceUUID;
            if (pService != nullptr) {
                serviceUUID = pService->getUUID();
            }
            BLEAdvertisementData freshAdvertisementData;
            static String savedDeviceName = "";
            if (savedDeviceName.length() == 0) {
                savedDeviceName = "OEPL" + getChipIdHex();
            }
            freshAdvertisementData.setName(savedDeviceName);
            freshAdvertisementData.setManufacturerData(manufacturerDataStr);
            *advertisementData = freshAdvertisementData;
            pAdvertising->setAdvertisementData(freshAdvertisementData);
            if (pService != nullptr) {
                pAdvertising->addServiceUUID(serviceUUID);
            }
            pAdvertising->setScanResponse(false);
            pAdvertising->setMinPreferred(0x06);
            pAdvertising->setMinPreferred(0x12);
            pAdvertising->start();
        } else {
            writeSerial("ERROR: Failed to get advertising object for update");
        }
} else {
    writeSerial("WARNING: updatemsdata called without advertisementData for ESP32");
}
#endif
mloopcounter++;
writeSerial("MSD data updated: " + String(mloopcounter));
}

void full_config_init(){
    writeSerial("Initializing config storage...");
    if (initConfigStorage()) {
        writeSerial("Config storage initialized successfully");
    } else {
        writeSerial("Config storage initialization failed");
    }
    writeSerial("Loading global configuration...");
    if (loadGlobalConfig()) {
        writeSerial("Global configuration loaded successfully");
        printConfigSummary();
        #ifdef TARGET_NRF
        if (globalConfig.loaded && (globalConfig.system_config.device_flags & DEVICE_FLAG_XIAOINIT)) {
            writeSerial("Device flag DEVICE_FLAG_XIAOINIT is set, calling xiaoinit()...");
            xiaoinit();
            writeSerial("xiaoinit() completed");
        }
        #endif
    } else {
       writeSerial("Global configuration load failed or no config found");
    }
}

void ble_init(){
    #ifdef TARGET_NRF
    Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.autoConnLed(false);
    Bluefruit.setTxPower(globalConfig.power_option.tx_power);
    Bluefruit.begin(1, 0);
    bledfu.begin();
    writeSerial("BLE DFU initialized successfully");
    writeSerial("BLE initialized successfully");
    writeSerial("Setting up BLE service 0x2446...");
    imageService.begin();
    writeSerial("BLE service started");
    imageCharacteristic.setWriteCallback(imageDataWritten);
    writeSerial("BLE write callback set");
    imageCharacteristic.begin();
    writeSerial("BLE characteristic started");
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
    writeSerial("BLE callbacks registered");
    String deviceName = "OEPL" + getChipIdHex();
    Bluefruit.setName(deviceName.c_str());
    writeSerial("Device name set to: " + deviceName);
    writeSerial("Configuring power management...");
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    writeSerial("Power management configured");
    writeSerial("Configuring BLE advertising...");
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    updatemsdata();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 200);
    Bluefruit.Advertising.setFastTimeout(10);
    writeSerial("Starting BLE advertising...");
    Bluefruit.Advertising.start(0);
    #endif
    #ifdef TARGET_ESP32
    writeSerial("=== Initializing ESP32 BLE ===");
    String deviceName = "OEPL" + getChipIdHex();
    writeSerial("Device name will be: " + deviceName);
    BLEDevice::init(deviceName.c_str());
    writeSerial("Setting BLE MTU to 512...");
    BLEDevice::setMTU(512);
    pServer = BLEDevice::createServer();
    if (pServer == nullptr) {
        writeSerial("ERROR: Failed to create BLE server");
        return;
    }
    pServer->setCallbacks(&staticServerCallbacks);
    writeSerial("Server callbacks configured");
    BLEUUID serviceUUID("00002446-0000-1000-8000-00805F9B34FB");
    pService = pServer->createService(serviceUUID);
    if (pService == nullptr) {
        writeSerial("ERROR: Failed to create BLE service");
        return;
    }
    writeSerial("BLE service 0x2446 created successfully");
    BLEUUID charUUID("00002446-0000-1000-8000-00805F9B34FB");
    pTxCharacteristic = pService->createCharacteristic(
        charUUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR
    );
    if (pTxCharacteristic == nullptr) {
        writeSerial("ERROR: Failed to create BLE characteristic");
        return;
    }
    writeSerial("Characteristic created with properties: READ, NOTIFY, WRITE, WRITE_NR");
    pTxCharacteristic->setCallbacks(&staticCharCallbacks);
    pRxCharacteristic = pTxCharacteristic;
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    if (pAdvertising == nullptr) {
        writeSerial("ERROR: Failed to get advertising object");
        return;
    }
    pAdvertising->addServiceUUID(serviceUUID);
    writeSerial("Service UUID added to advertising");
    // Use global advertisementData object (not a local variable)
    advertisementData->setName(deviceName);
    writeSerial("Device name added to advertising");
    // Add manufacturer data using updatemsdata function
    updatemsdata();
    pAdvertising->setAdvertisementData(*advertisementData);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0006);
    pAdvertising->setMinPreferred(0x0012);
    writeSerial("Advertising intervals set");
    pServer->getAdvertising()->setMinPreferred(0x06);
    pServer->getAdvertising()->setMinPreferred(0x12);
    pServer->getAdvertising()->start();
    writeSerial("=== BLE advertising started successfully ===");
    writeSerial("Device ready: " + deviceName);
    writeSerial("Waiting for BLE connections...");
#endif
}

void pwrmgm(bool onoff){
    if(globalConfig.display_count == 0){
        writeSerial("No display configured");
        return;
    }
    if(globalConfig.system_config.pwr_pin != 0xFF){
    #ifdef TARGET_ESP32
    digitalWrite(globalConfig.system_config.pwr_pin, HIGH);
    #endif
    #ifdef TARGET_NRF
    if(onoff){
        digitalWrite(globalConfig.system_config.pwr_pin, HIGH);
        pinMode(globalConfig.displays[0].reset_pin, OUTPUT);
        pinMode(globalConfig.displays[0].cs_pin, OUTPUT);
        pinMode(globalConfig.displays[0].dc_pin, OUTPUT);
        pinMode(globalConfig.displays[0].clk_pin, OUTPUT);
        pinMode(globalConfig.displays[0].data_pin, OUTPUT);
        delay(200);
    }
    else{
        SPI.end();
        Wire.end();
        pinMode(globalConfig.displays[0].reset_pin, INPUT);
        pinMode(globalConfig.displays[0].cs_pin, INPUT);
        pinMode(globalConfig.displays[0].dc_pin, INPUT);
        pinMode(globalConfig.displays[0].clk_pin, INPUT);
        pinMode(globalConfig.displays[0].data_pin, INPUT);
        digitalWrite(globalConfig.system_config.pwr_pin, LOW);
    }
    #endif
    }
   else{
    writeSerial("Power pin not set");
   }
}

void writeSerial(String message, bool newLine){
    #ifndef DISABLE_USB_SERIAL
    if (newLine == true) Serial.println(message);
    else Serial.print(message);
    #endif
    // When USB is disabled, writeSerial does nothing (no-op) to save power
}

void xiaoinit(){
    powerDownExternalFlash(20,24,21,25,22,23);
    pinMode(31, INPUT);
    pinMode(14, INPUT);
    pinMode(13, OUTPUT);  //that actually does something
    digitalWrite(13, LOW);
    pinMode(17, INPUT);
    //buttons
    pinMode(15, INPUT);
    pinMode(3, INPUT);
    pinMode(28, INPUT);
}

bool powerDownExternalFlash(uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin, uint8_t csPin, uint8_t wpPin, uint8_t holdPin) {
    #ifdef TARGET_NRF
    // Software SPI transfer function (bit-banging)
    auto spiTransfer = [&](uint8_t data) -> uint8_t {
        uint8_t result = 0;
        for (int i = 7; i >= 0; i--) {
            // Set MOSI to current bit
            digitalWrite(mosiPin, (data >> i) & 1);
            // Clock low
            digitalWrite(sckPin, LOW);
            delayMicroseconds(1);
            // Read MISO
            result |= (digitalRead(misoPin) << i);
            // Clock high
            digitalWrite(sckPin, HIGH);
            delayMicroseconds(1);
        }
        return result;
    };
    writeSerial("=== External Flash Power-Down ===");
    writeSerial("Pin configuration: MOSI=" + String(mosiPin) + " MISO=" + String(misoPin) + " SCK=" + String(sckPin) + " CS=" + String(csPin) + " WP=" + String(wpPin) + " HOLD=" + String(holdPin));
    writeSerial("Configuring SPI pins...");
    pinMode(mosiPin, OUTPUT);
    pinMode(misoPin, INPUT);
    pinMode(sckPin, OUTPUT);
    pinMode(csPin, OUTPUT);
    pinMode(wpPin, OUTPUT);
    pinMode(holdPin, OUTPUT);
    writeSerial("SPI pins configured");
    digitalWrite(sckPin, HIGH);  // Clock idle high (SPI mode 0)
    digitalWrite(csPin, HIGH);   // CS inactive
    digitalWrite(wpPin, HIGH);   // WP disabled (active-low)
    digitalWrite(holdPin, HIGH); // HOLD disabled (active-low)
    writeSerial("Control pins set: CS=HIGH, WP=HIGH (disabled), HOLD=HIGH (disabled), SCK=HIGH (idle)");
    delay(1);
    writeSerial("Attempting to wake flash from deep power-down (command 0xAB)...");
    digitalWrite(csPin, LOW);
    spiTransfer(0xAB);
    digitalWrite(csPin, HIGH);
    delay(10); // Wait for flash to wake up (typically 3-35us, using 10ms for safety)
    writeSerial("Wake-up command sent, waiting 10ms...");
    // Read JEDEC ID before power-down (for verification)
    writeSerial("Reading JEDEC ID before power-down...");
    digitalWrite(csPin, LOW);
    spiTransfer(0x9F); // JEDEC ID command
    uint8_t jedecId[3];
    for (int i = 0; i < 3; i++) {
        jedecId[i] = spiTransfer(0x00);
    }
    digitalWrite(csPin, HIGH);
    String jedecIdStr = "0x";
    for (int i = 0; i < 3; i++) {
        if (jedecId[i] < 16) jedecIdStr += "0";
        jedecIdStr += String(jedecId[i], HEX);
    }
    jedecIdStr.toUpperCase();
    writeSerial("JEDEC ID before: " + jedecIdStr + " (Manufacturer=0x" + String(jedecId[0], HEX) + ", MemoryType=0x" + String(jedecId[1], HEX) + ", Capacity=0x" + String(jedecId[2], HEX) + ")");
    bool flashResponding = false;
    for (int i = 0; i < 3; i++) {
        if (jedecId[i] != 0xFF) {
            flashResponding = true;
            break;
        }
    }
    delay(1);
    writeSerial("Sending deep power-down command (0xB9)...");
    digitalWrite(csPin, LOW);
    spiTransfer(0xB9);
    digitalWrite(csPin, HIGH);
    if(false){
    writeSerial("Deep power-down command sent, waiting 10ms...");
    delay(10); // Wait for command to complete
    writeSerial("Reading JEDEC ID after power-down command...");
    digitalWrite(csPin, LOW);
    spiTransfer(0x9F);
    uint8_t jedecIdAfter[3];
    for (int i = 0; i < 3; i++) {
        jedecIdAfter[i] = spiTransfer(0x00);
    }
    digitalWrite(csPin, HIGH);
    String jedecIdAfterStr = "0x";
    for (int i = 0; i < 3; i++) {
        if (jedecIdAfter[i] < 16) jedecIdAfterStr += "0";
        jedecIdAfterStr += String(jedecIdAfter[i], HEX);
    }
    jedecIdAfterStr.toUpperCase();
    writeSerial("JEDEC ID after: " + jedecIdAfterStr + " (byte[0]=0x" + String(jedecIdAfter[0], HEX) + ", byte[1]=0x" + String(jedecIdAfter[1], HEX) + ", byte[2]=0x" + String(jedecIdAfter[2], HEX) + ")");
    bool inPowerDown = true;
    String mismatchBytes = "";
    for (int i = 0; i < 3; i++) {
        if (jedecIdAfter[i] != 0xFF) {
            inPowerDown = false;
            if (mismatchBytes.length() > 0) mismatchBytes += ", ";
            mismatchBytes += "byte[" + String(i) + "]=0x" + String(jedecIdAfter[i], HEX) + " (expected 0xFF)";
        }
    }
    }
    digitalWrite(csPin, HIGH);
    pinMode(wpPin, INPUT);
    pinMode(holdPin, INPUT);
    pinMode(mosiPin, INPUT);
    pinMode(misoPin, INPUT);
    pinMode(sckPin, INPUT);    
    #else
    writeSerial("External flash power-down not implemented for ESP32");
    return false;
    #endif
    return false;
}

int mapEpd(int id){
    switch(id) {
        case 0x0000: return EP_PANEL_UNDEFINED; // ep_panel_undefined
        case 0x0001: return EP42_400x300; // ep42_400x300
        case 0x0002: return EP42B_400x300; // ep42b_400x300
        case 0x0003: return EP213_122x250; // ep213_122x250
        case 0x0004: return EP213B_122x250; // ep213b_122x250
        case 0x0005: return EP293_128x296; // ep293_128x296
        case 0x0006: return EP294_128x296; // ep294_128x296
        case 0x0007: return EP295_128x296; // ep295_128x296
        case 0x0008: return EP295_128x296_4GRAY; // ep295_128x296_4gray
        case 0x0009: return EP266_152x296; // ep266_152x296
        case 0x000A: return EP102_80x128; // ep102_80x128
        case 0x000B: return EP27B_176x264; // ep27b_176x264
        case 0x000C: return EP29R_128x296; // ep29r_128x296
        case 0x000D: return EP122_192x176; // ep122_192x176
        case 0x000E: return EP154R_152x152; // ep154r_152x152
        case 0x000F: return EP42R_400x300; // ep42r_400x300
        case 0x0010: return EP42R2_400x300; // ep42r2_400x300
        case 0x0011: return EP37_240x416; // ep37_240x416
        case 0x0012: return EP37B_240x416; // ep37b_240x416
        case 0x0013: return EP213_104x212; // ep213_104x212
        case 0x0014: return EP75_800x480; // ep75_800x480 (older version)
        case 0x0015: return EP75_800x480_4GRAY; // ep75_800x480_4gray (older version)
        case 0x0016: return EP75_800x480_4GRAY_V2; // ep75_800x480_4gray_v2 (renamed from ep75_800x480_4gray_old)
        case 0x0017: return EP29_128x296; // ep29_128x296
        case 0x0018: return EP29_128x296_4GRAY; // ep29_128x296_4gray
        case 0x0019: return EP213R_122x250; // ep213r_122x250
        case 0x001A: return EP154_200x200; // ep154_200x200
        case 0x001B: return EP154B_200x200; // ep154b_200x200
        case 0x001C: return EP266YR_184x360; // ep266yr_184x360
        case 0x001D: return EP29YR_128x296; // ep29yr_128x296
        case 0x001E: return EP29YR_168x384; // ep29yr_168x384
        case 0x001F: return EP583_648x480; // ep583_648x480
        case 0x0020: return EP296_128x296; // ep296_128x296
        case 0x0021: return EP26R_152x296; // ep26r_152x296
        case 0x0022: return EP73_800x480; // ep73_800x480
        case 0x0023: return EP73_SPECTRA_800x480; // ep73_spectra_800x480
        case 0x0024: return EP74R_640x384; // ep74r_640x384
        case 0x0025: return EP583R_600x448; // ep583r_600x448
        case 0x0026: return EP75R_800x480; // ep75r_800x480
        case 0x0027: return EP426_800x480; // ep426_800x480
        case 0x0028: return EP426_800x480_4GRAY; // ep426_800x480_4gray
        case 0x0029: return EP29R2_128x296; // ep29r2_128x296
        case 0x002A: return EP41_640x400; // ep41_640x400
        case 0x002B: return EP81_SPECTRA_1024x576; // ep81_spectra_1024x576
        case 0x002C: return EP7_960x640; // ep7_960x640
        case 0x002D: return EP213R2_122x250; // ep213r2_122x250
        case 0x002E: return EP29Z_128x296; // ep29z_128x296
        case 0x002F: return EP29Z_128x296_4GRAY; // ep29z_128x296_4gray
        case 0x0030: return EP213Z_122x250; // ep213z_122x250
        case 0x0031: return EP213Z_122x250_4GRAY; // ep213z_122x250_4gray
        case 0x0032: return EP154Z_152x152; // ep154z_152x152
        case 0x0033: return EP579_792x272; // ep579_792x272
        case 0x0034: return EP213YR_122x250; // ep213yr_122x250
        case 0x0035: return EP37YR_240x416; // ep37yr_240x416
        case 0x0036: return EP35YR_184x384; // ep35yr_184x384
        case 0x0037: return EP397YR_800x480; // ep397yr_800x480
        case 0x0038: return EP154YR_200x200; // ep154yr_200x200
        case 0x0039: return EP266YR2_184x360; // ep266yr2_184x360
        case 0x003A: return EP42YR_400x300; // ep42yr_400x300
        case 0x003B: return EP75_800x480_GEN2; // ep75_800x480_gen2
        case 0x003C: return EP75_800x480_4GRAY_GEN2; // ep75_800x480_4gray_gen2
        case 0x003D: return EP215YR_160x296; // ep215yr_160x296
        case 0x003E: return EP1085_1360x480; // ep1085_1360x480
        case 0x003F: return EP31_240x320; // ep31_240x320
        default: return EP_PANEL_UNDEFINED; // Unknown panel type
    }
}

void initDisplay(){
    writeSerial("=== Initializing Display ===");
    if(globalConfig.display_count > 0){
    pwrmgm(true);
    memset(&bbep, 0, sizeof(BBEPDISP));
    int panelType = mapEpd(globalConfig.displays[0].panel_ic_type);
    bbepSetPanelType(&bbep, panelType);
    bbepSetRotation(&bbep, globalConfig.displays[0].rotation * 90);
    bbepInitIO(&bbep, globalConfig.displays[0].dc_pin, globalConfig.displays[0].reset_pin, globalConfig.displays[0].busy_pin, globalConfig.displays[0].cs_pin, globalConfig.displays[0].data_pin, globalConfig.displays[0].clk_pin, 8000000);
    writeSerial(String("Height: ") + String(globalConfig.displays[0].pixel_height));
    writeSerial(String("Width: ") + String(globalConfig.displays[0].pixel_width));
    bbepWakeUp(&bbep);
    bbepSendCMDSequence(&bbep, bbep.pInitFull);
    String chipId = getChipIdHex();
    String infoText = "openepaperlink.org\nName: OEPL" + chipId + "\nFW: " + String(getFirmwareMajor()) + "." + String(getFirmwareMinor()) + "\nEpaper driver by\nLarry Bank\ngithub.com/bitbank2";
    if (globalConfig.displays[0].transmission_modes & TRANSMISSION_MODE_CLEAR_ON_BOOT) writeTextAndFill("");
    else writeTextAndFill(infoText.c_str());
    bbepRefresh(&bbep, REFRESH_FULL);
    waitforrefresh(60);
    pwrmgm(false);
    }
    else{
        writeSerial("No display found");
    }
}

bool waitforrefresh(int timeout){
    for (size_t i = 0; i < (size_t)(timeout * 10); i++){
        delay(100);
        if(i % 5 == 0)writeSerial(".",false);
        if(!bbepIsBusy(&bbep)){ 
        writeSerial(".");
        writeSerial("Refresh took ",false);
        writeSerial((String)((float)i / 10),false);
        writeSerial(" seconds");
        delay(200);
        return true;
        }
    }
    writeSerial("Refresh timed out");
    return false;
}

void connect_callback(uint16_t conn_handle) {
    writeSerial("=== BLE CLIENT CONNECTED ===");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;
    writeSerial("=== BLE CLIENT DISCONNECTED ===");
    writeSerial("Disconnect reason: " + String(reason));
}

String getChipIdHex() {
    #ifdef TARGET_NRF
    uint32_t id1 = NRF_FICR->DEVICEID[0];
    uint32_t id2 = NRF_FICR->DEVICEID[1]; 
    uint32_t last3Bytes = id2 & 0xFFFFFF;
    String hexId = String(last3Bytes, HEX);
    hexId.toUpperCase();
    while (hexId.length() < 6) {
        hexId = "0" + hexId;
    }
    writeSerial("Chip ID: " + String(id1, HEX) + String(id2, HEX));
    writeSerial("Using last 3 bytes: " + hexId);
    return hexId;
    #endif
    #ifdef TARGET_ESP32
    uint64_t macAddress = ESP.getEfuseMac();
    uint32_t chipId = (uint32_t)(macAddress >> 24) & 0xFFFFFF;
    String hexId = String(chipId, HEX);
    hexId.toUpperCase();
    while (hexId.length() < 6) {
        hexId = "0" + hexId;
    }
    writeSerial("Chip ID: " + String(chipId, HEX));
    writeSerial("Using chip ID: " + hexId);
    return hexId;
    #endif
}

void reboot(){
    writeSerial("=== REBOOT COMMAND (0x000F) ===");
    delay(100);
    #ifdef TARGET_NRF
    NVIC_SystemReset();
    #endif
    #ifdef TARGET_ESP32
    esp_restart();
    #endif
}

void sendResponse(uint8_t* response, uint8_t len){
    writeSerial("Sending BLE response:");
    writeSerial("  Length: " + String(len) + " bytes");
    writeSerial("  Command: 0x" + String(response[0], HEX) + String(response[1], HEX));
    String hexDump = "  Full command: ";
    for (int i = 0; i < len; i++) {
        if (i > 0) hexDump += " ";
        if (response[i] < 16) hexDump += "0";
        hexDump += String(response[i], HEX);
    }
    writeSerial(hexDump);
    #ifdef TARGET_NRF
    imageCharacteristic.notify(response, len);
    writeSerial("Response notified (nRF52)");
    #endif
    #ifdef TARGET_ESP32
    // Queue response to avoid calling notify() from callback context
    if (len <= MAX_RESPONSE_SIZE) {
        uint8_t nextHead = (responseQueueHead + 1) % RESPONSE_QUEUE_SIZE;
        if (nextHead != responseQueueTail) {
            memcpy(responseQueue[responseQueueHead].data, response, len);
            responseQueue[responseQueueHead].len = len;
            responseQueue[responseQueueHead].pending = true;
            responseQueueHead = nextHead;
            writeSerial("ESP32: Response queued (queue size: " + String((responseQueueHead - responseQueueTail + RESPONSE_QUEUE_SIZE) % RESPONSE_QUEUE_SIZE) + ")");
        } else {
            writeSerial("ERROR: Response queue full, dropping response");
        }
    } else {
        writeSerial("ERROR: Response too large for queue (" + String(len) + " > " + String(MAX_RESPONSE_SIZE) + ")");
    }
    #endif
    #ifndef TARGET_ESP32
    delay(20);
    writeSerial("Response sent successfully");
    #endif
}

bool initConfigStorage(){
    #ifdef TARGET_NRF
    if (!InternalFS.begin()) {
        writeSerial("ERROR: Failed to mount internal file system");
        return false;
    }
    return true;
    #endif
    #ifdef TARGET_ESP32
    if (!LittleFS.begin(true)) { // true = format on failure
        writeSerial("ERROR: Failed to mount LittleFS");
        return false;
    }
    return true;
    #endif
    return false; // Should never reach here
}

void formatConfigStorage(){
    #ifdef TARGET_NRF
    InternalFS.format();
    #endif
    #ifdef TARGET_ESP32
    LittleFS.format();
    #endif
}

bool saveConfig(uint8_t* configData, uint32_t len){
    if (len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Config data too large (" + String(len) + " bytes)");
        return false;
    }
    static config_storage_t config;
    config.magic = 0xDEADBEEF;
    config.version = 1;
    config.data_len = len;
    config.crc = calculateConfigCRC(configData, len);
    memcpy(config.data, configData, len);
    size_t headerSize = sizeof(config_storage_t) - MAX_CONFIG_SIZE; // Size without data array
    size_t totalSize = headerSize + len; // Header + actual data length
    #ifdef TARGET_NRF
    if (InternalFS.exists(CONFIG_FILE_PATH)) {
        InternalFS.remove(CONFIG_FILE_PATH);
    }
    File file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_WRITE);
    #elif defined(TARGET_ESP32)
    if (LittleFS.exists(CONFIG_FILE_PATH)) {
        LittleFS.remove(CONFIG_FILE_PATH);
    }
    File file = LittleFS.open(CONFIG_FILE_PATH, FILE_WRITE);
    #endif
    if (!file) {
        writeSerial("ERROR: Failed to open config file for writing");
        #ifdef TARGET_NRF
        file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_WRITE);
        #elif defined(TARGET_ESP32)
        file = LittleFS.open(CONFIG_FILE_PATH, FILE_WRITE);
        #endif
        if (!file) {
            writeSerial("ERROR: Failed to open config file for writing with CREATE|WRITE");
        return false;
        }
    }
    size_t bytesWritten = file.write((uint8_t*)&config, totalSize);
    file.close();
    if (bytesWritten != totalSize) {
        writeSerial("ERROR: Failed to write complete config data (expected " + String(totalSize) + ", wrote " + String(bytesWritten) + ")");
        return false;
    }
    return true;
}

bool loadConfig(uint8_t* configData, uint32_t* len){
    #ifdef TARGET_NRF
    File file = InternalFS.open(CONFIG_FILE_PATH, FILE_O_READ);
    #elif defined(TARGET_ESP32)
    File file = LittleFS.open(CONFIG_FILE_PATH, FILE_READ);
    #endif
    if (!file) {
        return false;
    }
    static config_storage_t config;
    static size_t bytesRead;
    static size_t headerSize = sizeof(config_storage_t) - MAX_CONFIG_SIZE; // Size without data array
    bytesRead = file.read((uint8_t*)&config, headerSize);
    if (bytesRead != headerSize) {
        writeSerial("ERROR: Failed to read config header (expected " + String(headerSize) + ", got " + String(bytesRead) + ")");
        file.close();
        return false;
    }
    if (config.magic != 0xDEADBEEF) {
        writeSerial("ERROR: Invalid config magic number");
        file.close();
        return false;
    }
    if (config.data_len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Config data too large");
        file.close();
        return false;
    }
    bytesRead = file.read(config.data, config.data_len);
    file.flush();
    file.close();
    if (bytesRead != config.data_len) {
        writeSerial("ERROR: Failed to read complete config data (expected " + String(config.data_len) + ", read " + String(bytesRead) + ")");
        return false;
    }
    uint32_t calculatedCRC = calculateConfigCRC(config.data, config.data_len);
    if (config.crc != calculatedCRC) {
        writeSerial("ERROR: Config CRC mismatch");
        return false;
    }
    if (config.data_len > *len) {
        writeSerial("ERROR: Config data larger than buffer");
        return false;
    }
    for (uint32_t i = 0; i < config.data_len && i < *len; i++) {
        configData[i] = config.data[i];
    }
    *len = config.data_len;
    return true;
}

uint32_t calculateConfigCRC(uint8_t* data, uint32_t len){
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return ~crc;
}

void handleReadConfig(){
    uint8_t configData[MAX_CONFIG_SIZE];
    uint32_t configLen = MAX_CONFIG_SIZE;
    if (loadConfig(configData, &configLen)) {
        writeSerial("Sending config data in chunks...");
        uint32_t remaining = configLen;
        uint32_t offset = 0;
        uint16_t chunkNumber = 0;
        const uint16_t maxChunks = 10;
        while (remaining > 0 && chunkNumber < maxChunks) {
            uint16_t responseLen = 0;
            configReadResponseBuffer[responseLen++] = 0x00; // Response type
            configReadResponseBuffer[responseLen++] = 0x40; // Command echo
            configReadResponseBuffer[responseLen++] = chunkNumber & 0xFF;
            configReadResponseBuffer[responseLen++] = (chunkNumber >> 8) & 0xFF;
            if (chunkNumber == 0) {
                configReadResponseBuffer[responseLen++] = configLen & 0xFF;
                configReadResponseBuffer[responseLen++] = (configLen >> 8) & 0xFF;
            }
            uint16_t maxDataSize = MAX_RESPONSE_DATA_SIZE - responseLen;
            uint16_t chunkSize = (remaining < maxDataSize) ? remaining : maxDataSize;
            if (chunkSize == 0) {
                writeSerial("ERROR: Chunk size is 0, breaking");
                break;
            }
            memcpy(configReadResponseBuffer + responseLen, configData + offset, chunkSize);
            responseLen += chunkSize;
            if (responseLen > 100) {
                writeSerial("ERROR: Response too large (" + String(responseLen) + " bytes), skipping");
                break;
            }
            if (responseLen == 0) {
                writeSerial("ERROR: Empty response, skipping");
                break;
            }
            sendResponse(configReadResponseBuffer, responseLen);
            offset += chunkSize;
            remaining -= chunkSize;
            chunkNumber++;
            writeSerial("Sent chunk " + String(chunkNumber) + " (" + String(chunkSize) + " bytes)");
            delay(50);
        }
        if (chunkNumber >= maxChunks) {
            writeSerial("WARNING: Hit chunk limit, transmission may be incomplete");
        }
        writeSerial("Config read response sent (" + String(configLen) + " bytes) in " + String(chunkNumber) + " chunks");
    } else {
        uint8_t errorResponse[] = {0xFF, 0x40, 0x00, 0x00}; // Error, command, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        writeSerial("Config read failed - sent error response");
    }
    writeSerial("About to return from handleReadConfig");
    delay(100);
    writeSerial("handleReadConfig function completed successfully");
}

void handleFirmwareVersion(){
    writeSerial("Building Firmware Version response...");
    uint8_t major = getFirmwareMajor();
    uint8_t minor = getFirmwareMinor();
    const char* shaCStr = SHA_STRING;
    String shaStr = String(shaCStr);
    if (shaStr.length() >= 2 && shaStr.charAt(0) == '"' && shaStr.charAt(shaStr.length() - 1) == '"') {
        shaStr = shaStr.substring(1, shaStr.length() - 1);
    }
    writeSerial("Firmware version: " + String(major) + "." + String(minor));
    writeSerial("SHA: " + shaStr);
    uint8_t shaLen = shaStr.length();
    if (shaLen > 40) shaLen = 40;
    uint8_t response[2 + 1 + 1 + 1 + 40];
    uint16_t offset = 0;
    response[offset++] = 0x00;
    response[offset++] = 0x43;
    response[offset++] = major;
    response[offset++] = minor;
    response[offset++] = shaLen;
    for (uint8_t i = 0; i < shaLen && i < 40; i++) {
        response[offset++] = shaStr.charAt(i);
    }
    sendResponse(response, offset);
    writeSerial("Firmware version response sent");
}

void handleWriteConfig(uint8_t* data, uint16_t len){
    if (len == 0) {
        writeSerial("ERROR: No config data received");
        return;
    }
    if (len > CONFIG_CHUNK_SIZE) {
        writeSerial("Starting chunked write (received: " + String(len) + " bytes)");
        chunkedWriteState.active = true;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.expectedChunks = 0;
        chunkedWriteState.receivedChunks = 0;
        if (len >= CONFIG_CHUNK_SIZE_WITH_PREFIX) {
            chunkedWriteState.totalSize = data[0] | (data[1] << 8);
            chunkedWriteState.expectedChunks = (chunkedWriteState.totalSize + CONFIG_CHUNK_SIZE - 1) / CONFIG_CHUNK_SIZE;
            uint16_t chunkDataSize = ((len - 2) < CONFIG_CHUNK_SIZE) ? (len - 2) : CONFIG_CHUNK_SIZE;
            memcpy(chunkedWriteState.buffer, data + 2, chunkDataSize);
            chunkedWriteState.receivedSize = chunkDataSize;
            chunkedWriteState.receivedChunks = 1;
            writeSerial("First chunk received: " + String(chunkDataSize) + " bytes, total size: " + String(chunkedWriteState.totalSize) + " bytes, expected chunks: " + String(chunkedWriteState.expectedChunks));
        } else {
            chunkedWriteState.totalSize = len;
            chunkedWriteState.expectedChunks = 1;
            uint16_t chunkSize = (len < CONFIG_CHUNK_SIZE) ? len : CONFIG_CHUNK_SIZE;
            memcpy(chunkedWriteState.buffer, data, chunkSize);
            chunkedWriteState.receivedSize = chunkSize;
            chunkedWriteState.receivedChunks = 1;
            writeSerial("Large single transmission: " + String(chunkSize) + " bytes");
        }
        uint8_t ackResponse[] = {0x00, 0x41, 0x00, 0x00}; // Success, command, chunk received
        sendResponse(ackResponse, sizeof(ackResponse));
        return;
    }
    if (saveConfig(data, len)) {
        uint8_t successResponse[] = {0x00, 0x41, 0x00, 0x00}; // Success, command, no data
        sendResponse(successResponse, sizeof(successResponse));
        writeSerial("Config write successful");
    } else {
        uint8_t errorResponse[] = {0xFF, 0x41, 0x00, 0x00}; // Error, command, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        writeSerial("Config write failed");
    }
}

void handleWriteConfigChunk(uint8_t* data, uint16_t len){
    if (!chunkedWriteState.active) {
        writeSerial("ERROR: No chunked write in progress");
        uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    if (len == 0) {
        writeSerial("ERROR: No chunk data received");
        return;
    }
    if (len > CONFIG_CHUNK_SIZE) {
        writeSerial("ERROR: Chunk too large (" + String(len) + " bytes)");
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    if (chunkedWriteState.receivedSize + len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Chunk would exceed max config size");
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00}; // Error, command, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    if (chunkedWriteState.receivedChunks >= MAX_CONFIG_CHUNKS) {
        writeSerial("ERROR: Too many chunks received");
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    memcpy(chunkedWriteState.buffer + chunkedWriteState.receivedSize, data, len);
    chunkedWriteState.receivedSize += len;
    chunkedWriteState.receivedChunks++;
    writeSerial("Chunk " + String(chunkedWriteState.receivedChunks) + "/" + String(chunkedWriteState.expectedChunks) + " received (" + String(len) + " bytes)");
    if (chunkedWriteState.receivedChunks >= chunkedWriteState.expectedChunks) {
        writeSerial("All chunks received, saving config (" + String(chunkedWriteState.receivedSize) + " bytes)");
        if (saveConfig(chunkedWriteState.buffer, chunkedWriteState.receivedSize)) {
            uint8_t successResponse[] = {0x00, 0x42, 0x00, 0x00}; // Success, command, no data
            sendResponse(successResponse, sizeof(successResponse));
            writeSerial("Chunked config write successful");
    } else {
            uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00}; // Error, command, no data
            sendResponse(errorResponse, sizeof(errorResponse));
            writeSerial("Chunked config write failed");
        }
        chunkedWriteState.active = false;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.receivedChunks = 0;
    } else {
        uint8_t ackResponse[] = {0x00, 0x42, 0x00, 0x00}; // Success, command, chunk received
        sendResponse(ackResponse, sizeof(ackResponse));
    }
}

bool loadGlobalConfig(){
    memset(&globalConfig, 0, sizeof(globalConfig));
    wifiConfigured = false;
    wifiSsid[0] = '\0';
    wifiPassword[0] = '\0';
    wifiEncryptionType = 0;
    static uint8_t configData[MAX_CONFIG_SIZE];
    static uint32_t configLen = MAX_CONFIG_SIZE;
    if (!loadConfig(configData, &configLen)) {
        globalConfig.loaded = false;
        return false;
    }
    if (configLen < 3) {
        writeSerial("ERROR: Config too short");
        globalConfig.loaded = false;
        return false;
    }
    uint32_t offset = 0;
    offset += 2;
    globalConfig.version = configData[offset++];
    globalConfig.minor_version = 0; // Not stored in current format
    while (offset < configLen - 2) { // -2 for CRC
        if (offset + 2 > configLen - 2) break;
        offset++;
        uint8_t packetId = configData[offset++];
        switch (packetId) {
            case 0x01: // system_config
                if (offset + sizeof(struct SystemConfig) <= configLen - 2) {
                    memcpy(&globalConfig.system_config, &configData[offset], sizeof(struct SystemConfig));
                    offset += sizeof(struct SystemConfig);
                } else {
                    writeSerial("ERROR: Not enough data for system_config");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x02: // manufacturer_data
                if (offset + sizeof(struct ManufacturerData) <= configLen - 2) {
                    memcpy(&globalConfig.manufacturer_data, &configData[offset], sizeof(struct ManufacturerData));
                    offset += sizeof(struct ManufacturerData);
                } else {
                    writeSerial("ERROR: Not enough data for manufacturer_data");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x04: // power_option
                if (offset + sizeof(struct PowerOption) <= configLen - 2) {
                    memcpy(&globalConfig.power_option, &configData[offset], sizeof(struct PowerOption));
                    offset += sizeof(struct PowerOption);
                } else {
                    writeSerial("ERROR: Not enough data for power_option");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x20: // display
                if (globalConfig.display_count < 4 && offset + sizeof(struct DisplayConfig) <= configLen - 2) {
                    memcpy(&globalConfig.displays[globalConfig.display_count], &configData[offset], sizeof(struct DisplayConfig));
                    offset += sizeof(struct DisplayConfig);
                    globalConfig.display_count++;
                } else if (globalConfig.display_count >= 4) {
                    writeSerial("WARNING: Maximum display count reached, skipping");
                    offset += sizeof(struct DisplayConfig);
                } else {
                    writeSerial("ERROR: Not enough data for display");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x21: // led
                if (globalConfig.led_count < 4 && offset + sizeof(struct LedConfig) <= configLen - 2) {
                    memcpy(&globalConfig.leds[globalConfig.led_count], &configData[offset], sizeof(struct LedConfig));
                    offset += sizeof(struct LedConfig);
                    globalConfig.led_count++;
                } else if (globalConfig.led_count >= 4) {
                    writeSerial("WARNING: Maximum LED count reached, skipping");
                    offset += sizeof(struct LedConfig);
                } else {
                    writeSerial("ERROR: Not enough data for LED");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x23: // sensor_data
                if (globalConfig.sensor_count < 4 && offset + sizeof(struct SensorData) <= configLen - 2) {
                    memcpy(&globalConfig.sensors[globalConfig.sensor_count], &configData[offset], sizeof(struct SensorData));
                    offset += sizeof(struct SensorData);
                    globalConfig.sensor_count++;
                } else if (globalConfig.sensor_count >= 4) {
                    writeSerial("WARNING: Maximum sensor count reached, skipping");
                    offset += sizeof(struct SensorData);
                } else {
                    writeSerial("ERROR: Not enough data for sensor");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x24: // data_bus
                if (globalConfig.data_bus_count < 4 && offset + sizeof(struct DataBus) <= configLen - 2) {
                    memcpy(&globalConfig.data_buses[globalConfig.data_bus_count], &configData[offset], sizeof(struct DataBus));
                    offset += sizeof(struct DataBus);
                    globalConfig.data_bus_count++;
                } else if (globalConfig.data_bus_count >= 4) {
                    writeSerial("WARNING: Maximum data_bus count reached, skipping");
                    offset += sizeof(struct DataBus);
                } else {
                    writeSerial("ERROR: Not enough data for data_bus");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x25: // binary_inputs
                if (globalConfig.binary_input_count < 4 && offset + sizeof(struct BinaryInputs) <= configLen - 2) {
                    memcpy(&globalConfig.binary_inputs[globalConfig.binary_input_count], &configData[offset], sizeof(struct BinaryInputs));
                    offset += sizeof(struct BinaryInputs);
                    globalConfig.binary_input_count++;
                } else if (globalConfig.binary_input_count >= 4) {
                    writeSerial("WARNING: Maximum binary_input count reached, skipping");
                    offset += sizeof(struct BinaryInputs);
                } else {
                    writeSerial("ERROR: Not enough data for binary_input");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
            case 0x26: // wifi_config
                {
                    const uint16_t WIFI_CONFIG_SIZE = 162;
                    if (offset + WIFI_CONFIG_SIZE <= configLen) {
                        memcpy(wifiSsid, &configData[offset], 32);
                        wifiSsid[32] = '\0';  // Ensure null termination
                        uint8_t ssidLen = 0;
                        while (ssidLen < 32 && wifiSsid[ssidLen] != '\0') ssidLen++;
                        offset += 32;
                        memcpy(wifiPassword, &configData[offset], 32);
                        wifiPassword[32] = '\0';  // Ensure null termination
                        uint8_t passwordLen = 0;
                        while (passwordLen < 32 && wifiPassword[passwordLen] != '\0') passwordLen++;
                        offset += 32;
                        wifiEncryptionType = configData[offset++];
                        offset += 95;
                        wifiConfigured = true;
                        writeSerial("=== WiFi Configuration Loaded ===");
                        writeSerial("SSID: \"" + String(wifiSsid) + "\"");
                        writeSerial("Password: " + String(passwordLen > 0 ? "***" : "(empty)"));
                        String encTypeStr = "Unknown";
                        switch(wifiEncryptionType) {
                            case 0x00: encTypeStr = "None (Open)"; break;
                            case 0x01: encTypeStr = "WEP"; break;
                            case 0x02: encTypeStr = "WPA"; break;
                            case 0x03: encTypeStr = "WPA2"; break;
                            case 0x04: encTypeStr = "WPA3"; break;
                        }
                        writeSerial("Encryption Type: 0x" + String(wifiEncryptionType, HEX) + " (" + encTypeStr + ")");
                        writeSerial("SSID length: " + String(ssidLen) + " bytes");
                        writeSerial("Password length: " + String(passwordLen) + " bytes");
                        writeSerial("WiFi configured: true");
                    } else {
                        writeSerial("ERROR: Not enough data for wifi_config");
                        globalConfig.loaded = false;
                        return false;
                    }
                }
                break;
            default:
                writeSerial("WARNING: Unknown packet ID 0x" + String(packetId, HEX) + ", skipping");
                offset = configLen - 2; // Skip to CRC
                break;
        }
    }
    if (offset < configLen - 2) {
        uint16_t crcGiven = configData[configLen - 2] | (configData[configLen - 1] << 8);
        uint32_t crcCalculated32 = calculateConfigCRC(configData, configLen - 2);
        uint16_t crcCalculated = (uint16_t)(crcCalculated32 & 0xFFFF);  // Use lower 16 bits for backwards compatibility
        if (crcGiven != crcCalculated) {
            writeSerial("WARNING: Config CRC mismatch (given: 0x" + String(crcGiven, HEX) + 
                       ", calculated: 0x" + String(crcCalculated, HEX) + ")");
        }
    }
    globalConfig.loaded = true;
    return true;
}

void printConfigSummary(){
    if (!globalConfig.loaded) {
        writeSerial("Config not loaded");
        return;
    }
    writeSerial("=== Configuration Summary ===");
    writeSerial("Version: " + String(globalConfig.version) + "." + String(globalConfig.minor_version));
    writeSerial("Loaded: " + String(globalConfig.loaded ? "Yes" : "No"));
    writeSerial("");
    // System Config
    writeSerial("--- System Configuration ---");
    writeSerial("IC Type: 0x" + String(globalConfig.system_config.ic_type, HEX));
    writeSerial("Communication Modes: 0x" + String(globalConfig.system_config.communication_modes, HEX));
    writeSerial("Device Flags: 0x" + String(globalConfig.system_config.device_flags, HEX));
    writeSerial("  PWR_PIN flag: " + String((globalConfig.system_config.device_flags & DEVICE_FLAG_PWR_PIN) ? "enabled" : "disabled"));
    #ifdef TARGET_NRF
    writeSerial("  XIAOINIT flag: " + String((globalConfig.system_config.device_flags & DEVICE_FLAG_XIAOINIT) ? "enabled" : "disabled"));
    #endif
    writeSerial("Power Pin: " + String(globalConfig.system_config.pwr_pin));
    writeSerial("");
    // Manufacturer Data
    writeSerial("--- Manufacturer Data ---");
    writeSerial("Manufacturer ID: 0x" + String(globalConfig.manufacturer_data.manufacturer_id, HEX));
    writeSerial("Board Type: " + String(globalConfig.manufacturer_data.board_type));
    writeSerial("Board Revision: " + String(globalConfig.manufacturer_data.board_revision));
    writeSerial("");
    // Power Option
    writeSerial("--- Power Configuration ---");
    writeSerial("Power Mode: " + String(globalConfig.power_option.power_mode));
    writeSerial("Battery Capacity: " + String(globalConfig.power_option.battery_capacity_mah[0]) + 
               " " + String(globalConfig.power_option.battery_capacity_mah[1]) + 
               " " + String(globalConfig.power_option.battery_capacity_mah[2]) + " mAh");
    writeSerial("Sleep Timeout: " + String(globalConfig.power_option.sleep_timeout_ms) + " ms");
    writeSerial("TX Power: " + String(globalConfig.power_option.tx_power));
    writeSerial("Sleep Flags: 0x" + String(globalConfig.power_option.sleep_flags, HEX));
    writeSerial("Battery Sense Pin: " + String(globalConfig.power_option.battery_sense_pin));
    writeSerial("Battery Sense Enable Pin: " + String(globalConfig.power_option.battery_sense_enable_pin));
    writeSerial("Battery Sense Flags: 0x" + String(globalConfig.power_option.battery_sense_flags, HEX));
    writeSerial("Capacity Estimator: " + String(globalConfig.power_option.capacity_estimator));
    writeSerial("Voltage Scaling Factor: " + String(globalConfig.power_option.voltage_scaling_factor));
    writeSerial("Deep Sleep Current: " + String(globalConfig.power_option.deep_sleep_current_ua) + " uA");
    writeSerial("");
    // Displays
    writeSerial("--- Display Configurations (" + String(globalConfig.display_count) + ") ---");
    for (int i = 0; i < globalConfig.display_count; i++) {
        writeSerial("Display " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.displays[i].instance_number));
        writeSerial("  Technology: 0x" + String(globalConfig.displays[i].display_technology, HEX));
        writeSerial("  Panel IC Type: 0x" + String(globalConfig.displays[i].panel_ic_type, HEX));
        writeSerial("  Resolution: " + String(globalConfig.displays[i].pixel_width) + "x" + String(globalConfig.displays[i].pixel_height));
        writeSerial("  Size: " + String(globalConfig.displays[i].active_width_mm) + "x" + String(globalConfig.displays[i].active_height_mm) + " mm");
        writeSerial("  OEPL Tag Type: 0x" + String(globalConfig.displays[i].oepl_tagtype, HEX));
        writeSerial("  Rotation: " + String(globalConfig.displays[i].rotation * 90) + " degrees");
        writeSerial("  Reset Pin: " + String(globalConfig.displays[i].reset_pin));
        writeSerial("  Busy Pin: " + String(globalConfig.displays[i].busy_pin));
        writeSerial("  DC Pin: " + String(globalConfig.displays[i].dc_pin));
        writeSerial("  CS Pin: " + String(globalConfig.displays[i].cs_pin));
        writeSerial("  Data Pin: " + String(globalConfig.displays[i].data_pin));
        writeSerial("  Partial Update: " + String(globalConfig.displays[i].partial_update_support ? "Yes" : "No"));
        writeSerial("  Color Scheme: 0x" + String(globalConfig.displays[i].color_scheme, HEX));
        writeSerial("  Transmission Modes: 0x" + String(globalConfig.displays[i].transmission_modes, HEX));
        writeSerial("    RAW: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_RAW) ? "enabled" : "disabled"));
        writeSerial("    ZIP: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_ZIP) ? "enabled" : "disabled"));
        writeSerial("    G5: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_G5) ? "enabled" : "disabled"));
        writeSerial("    DIRECT_WRITE: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_DIRECT_WRITE) ? "enabled" : "disabled"));
        writeSerial("    CLEAR_ON_BOOT: " + String((globalConfig.displays[i].transmission_modes & TRANSMISSION_MODE_CLEAR_ON_BOOT) ? "enabled" : "disabled"));
        writeSerial("");
    }
    // LEDs
    writeSerial("--- LED Configurations (" + String(globalConfig.led_count) + ") ---");
    for (int i = 0; i < globalConfig.led_count; i++) {
        writeSerial("LED " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.leds[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.leds[i].led_type, HEX));
        writeSerial("  Pins: R=" + String(globalConfig.leds[i].led_1_r) + 
                   " G=" + String(globalConfig.leds[i].led_2_g) + 
                   " B=" + String(globalConfig.leds[i].led_3_b) + 
                   " 4=" + String(globalConfig.leds[i].led_4));
        writeSerial("  Flags: 0x" + String(globalConfig.leds[i].led_flags, HEX));
        writeSerial("");
    }
    // Sensors
    writeSerial("--- Sensor Configurations (" + String(globalConfig.sensor_count) + ") ---");
    for (int i = 0; i < globalConfig.sensor_count; i++) {
        writeSerial("Sensor " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.sensors[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.sensors[i].sensor_type, HEX));
        writeSerial("  Bus ID: " + String(globalConfig.sensors[i].bus_id));
        writeSerial("");
    }
    // Data Buses
    writeSerial("--- Data Bus Configurations (" + String(globalConfig.data_bus_count) + ") ---");
    for (int i = 0; i < globalConfig.data_bus_count; i++) {
        writeSerial("Data Bus " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.data_buses[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.data_buses[i].bus_type, HEX));
        writeSerial("  Pins: 1=" + String(globalConfig.data_buses[i].pin_1) + 
                   " 2=" + String(globalConfig.data_buses[i].pin_2) + 
                   " 3=" + String(globalConfig.data_buses[i].pin_3) + 
                   " 4=" + String(globalConfig.data_buses[i].pin_4) + 
                   " 5=" + String(globalConfig.data_buses[i].pin_5) + 
                   " 6=" + String(globalConfig.data_buses[i].pin_6) + 
                   " 7=" + String(globalConfig.data_buses[i].pin_7));
        writeSerial("  Speed: " + String(globalConfig.data_buses[i].bus_speed_hz) + " Hz");
        writeSerial("  Flags: 0x" + String(globalConfig.data_buses[i].bus_flags, HEX));
        writeSerial("  Pullups: 0x" + String(globalConfig.data_buses[i].pullups, HEX));
        writeSerial("  Pulldowns: 0x" + String(globalConfig.data_buses[i].pulldowns, HEX));
        writeSerial("");
    }
    // Binary Inputs
    writeSerial("--- Binary Input Configurations (" + String(globalConfig.binary_input_count) + ") ---");
    for (int i = 0; i < globalConfig.binary_input_count; i++) {
        writeSerial("Binary Input " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.binary_inputs[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.binary_inputs[i].input_type, HEX));
        writeSerial("  Display As: 0x" + String(globalConfig.binary_inputs[i].display_as, HEX));
        writeSerial("  Pins: 1=" + String(globalConfig.binary_inputs[i].reserved_pin_1) + 
                   " 2=" + String(globalConfig.binary_inputs[i].reserved_pin_2) + 
                   " 3=" + String(globalConfig.binary_inputs[i].reserved_pin_3) + 
                   " 4=" + String(globalConfig.binary_inputs[i].reserved_pin_4) + 
                   " 5=" + String(globalConfig.binary_inputs[i].reserved_pin_5) + 
                   " 6=" + String(globalConfig.binary_inputs[i].reserved_pin_6) + 
                   " 7=" + String(globalConfig.binary_inputs[i].reserved_pin_7) + 
                   " 8=" + String(globalConfig.binary_inputs[i].reserved_pin_8));
        writeSerial("  Input Flags: 0x" + String(globalConfig.binary_inputs[i].input_flags, HEX));
        writeSerial("  Invert: 0x" + String(globalConfig.binary_inputs[i].invert, HEX));
        writeSerial("  Pullups: 0x" + String(globalConfig.binary_inputs[i].pullups, HEX));
        writeSerial("  Pulldowns: 0x" + String(globalConfig.binary_inputs[i].pulldowns, HEX));
        writeSerial("");
    }
    writeSerial("=============================");
}

float readBatteryVoltage() {
    if (globalConfig.power_option.battery_sense_pin == 0xFF) {
        return -1.0;
    }
    uint8_t sensePin = globalConfig.power_option.battery_sense_pin;
    uint8_t enablePin = globalConfig.power_option.battery_sense_enable_pin;
    uint16_t scalingFactor = globalConfig.power_option.voltage_scaling_factor;
    pinMode(sensePin, INPUT);
    if (enablePin != 0xFF) {
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, HIGH);
        delay(10);
    }
    const int numSamples = 10;
    uint32_t adcSum = 0;
    for (int i = 0; i < numSamples; i++) {
        adcSum += analogRead(sensePin);
        delay(2);
    }
    uint32_t adcAverage = adcSum / numSamples;
    if (enablePin != 0xFF) {
        digitalWrite(enablePin, LOW);
    }
    float voltage = -1.0;
    if (scalingFactor > 0) {
        voltage = (adcAverage * scalingFactor) / (100000.0);
    }
     return voltage;
}

float readChipTemperature() {
    #ifdef TARGET_ESP32
    float temp = temperatureRead();
    return temp; 
    #elif defined(TARGET_NRF)
    int32_t tempRaw = 0;
    uint32_t err_code = sd_temp_get(&tempRaw);
    if (err_code == 0) {
        float tempC = tempRaw * 0.25;
        return tempC;
    }
    return -999.0; // Fallback if SoftDevice API fails
    #else
    return -999.0;
    #endif
}

void handleDirectWriteStart(uint8_t* data, uint16_t len) {
    writeSerial("=== DIRECT WRITE START ===");
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    directWriteBitplanes = (colorScheme == 1 || colorScheme == 2); // BWR/BWY use bitplanes
    directWritePlane2 = false; // Start with plane 1
    directWriteCompressed = (len >= 4);
    if (directWriteCompressed) {
        memcpy(&directWriteDecompressedTotal, data, 4);
        writeSerial("Compressed direct write mode");
        writeSerial("Expected decompressed size: " + String(directWriteDecompressedTotal) + " bytes");
        directWriteWidth = globalConfig.displays[0].pixel_width;
        directWriteHeight = globalConfig.displays[0].pixel_height;
        if (directWriteBitplanes) {
            // Bitplanes: each plane is 1BPP, total data is 2x plane size
            directWriteTotalBytes = (directWriteWidth * directWriteHeight + 7) / 8; // Bytes per plane
            writeSerial("Bitplane mode: " + String(directWriteTotalBytes) + " bytes per plane, " + String(directWriteTotalBytes * 2) + " total");
        } else {
            int bitsPerPixel = getBitsPerPixel();
            if (bitsPerPixel == 4) {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 1) / 2; // 2 pixels per byte
            } else if (bitsPerPixel == 2) {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 3) / 4; // 4 pixels per byte
            } else {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 7) / 8; // 8 pixels per byte
            }
        }
        directWriteCompressedBuffer = compressedDataBuffer;
        directWriteCompressedSize = 0;  // Unknown until we receive all data
        directWriteCompressedReceived = 0;
        if (len > 4) {
            uint32_t compressedDataLen = len - 4;
            if (compressedDataLen <= MAX_IMAGE_SIZE) {
                memcpy(directWriteCompressedBuffer, data + 4, compressedDataLen);
                directWriteCompressedReceived = compressedDataLen;
                writeSerial("Initial compressed data: " + String(compressedDataLen) + " bytes");
            } else {
                writeSerial("ERROR: Initial compressed data too large for static buffer (" + String(compressedDataLen) + " > " + String(MAX_IMAGE_SIZE) + ")");
                writeSerial("Rejecting compressed upload - client should use uncompressed mode");
                directWriteActive = false;
                directWriteCompressed = false;
                uint8_t errorResponse[] = {0xFF, 0xFF};  // Error response: use uncompressed upload
                sendResponse(errorResponse, sizeof(errorResponse));
                return;
            }
        }
    } else {
        directWriteWidth = globalConfig.displays[0].pixel_width;
        directWriteHeight = globalConfig.displays[0].pixel_height;
        if (directWriteBitplanes) {
            directWriteTotalBytes = (directWriteWidth * directWriteHeight + 7) / 8; // Bytes per plane
            writeSerial("Bitplane mode: " + String(directWriteTotalBytes) + " bytes per plane, " + String(directWriteTotalBytes * 2) + " total");
        } else {
            int bitsPerPixel = getBitsPerPixel();
            if (bitsPerPixel == 4) {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 1) / 2; // 2 pixels per byte
            } else if (bitsPerPixel == 2) {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 3) / 4; // 4 pixels per byte
            } else {
                directWriteTotalBytes = (directWriteWidth * directWriteHeight + 7) / 8; // 8 pixels per byte
            }
        }
    }
    writeSerial("Display dimensions: " + String(directWriteWidth) + "x" + String(directWriteHeight));
    writeSerial("Expected total bytes: " + String(directWriteTotalBytes) + (directWriteBitplanes ? " per plane" : ""));
    directWriteActive = true;
    directWriteBytesWritten = 0;
    pwrmgm(true);
    bbepInitIO(&bbep, globalConfig.displays[0].dc_pin, globalConfig.displays[0].reset_pin, globalConfig.displays[0].busy_pin, globalConfig.displays[0].cs_pin, globalConfig.displays[0].data_pin, globalConfig.displays[0].clk_pin, 8000000);
    bbepWakeUp(&bbep);
    bbepSendCMDSequence(&bbep, bbep.pInitFull);// important for some displays
    bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
    bbepStartWrite(&bbep, directWriteBitplanes ? PLANE_0 : getplane());
    uint8_t ackResponse[] = {0x00, 0x70};
    sendResponse(ackResponse, sizeof(ackResponse));
    writeSerial("Direct write mode started, ready for data");
}

int getplane() {
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    if (colorScheme == 0) {
        return PLANE_0; // B/W uses PLANE_0
    } else if (colorScheme == 1 || colorScheme == 2) {
        return PLANE_0; // BWR/BWY use PLANE_0 (but use bitplanes for direct write)
    } else if (colorScheme == 5) {
        return PLANE_1; // 4 grayscale uses PLANE_1 with 2BPP (like BWRY)
    }
    else {
        return PLANE_1; // BWRY and 6-color use PLANE_1
    }
}

int getBitsPerPixel() {
    if (globalConfig.displays[0].color_scheme == 4) {
        return 4; // 4 bits per pixel (2 pixels per byte)
    }
    if (globalConfig.displays[0].color_scheme == 3) {
        return 2; // 2 bits per pixel (4 pixels per byte) for 3-4 color displays
    }
    if (globalConfig.displays[0].color_scheme == 5) {
        return 2; // 2 bits per pixel (4 pixels per byte) for 4 grayscale
    }
    return 1; // 1 bit per pixel (8 pixels per byte)
}

static void renderChar_4BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, int fontScale) {
    for (int col = 0; col < charWidth; col += fontScale) {
        uint8_t fontByte;
        int fontCol = col / fontScale;
        if (fontCol == 0 || fontCol > 7) {
            fontByte = 0x00;
        } else {
            fontByte = fontData[fontCol - 1];
        }
        uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
        uint8_t pixelNibble = (pixelBit == 1) ? 0x0 : 0xF;  // 1=black=0x0, 0=white=0xF
        for (int s = 0; s < fontScale; s++) {
            int pixelX = startX + charIdx * charWidth + col + s;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            int bytePos = pixelX / 2;
            if (bytePos >= pitch) break;
            if ((pixelX % 2) == 0) {
                rowBuffer[bytePos] = (rowBuffer[bytePos] & 0x0F) | (pixelNibble << 4);
            } else {
                rowBuffer[bytePos] = (rowBuffer[bytePos] & 0xF0) | pixelNibble;
            }
        }
    }
}

static void renderChar_2BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, uint8_t colorScheme, int fontScale) {
    uint8_t whiteCode = (colorScheme == 5) ? 0x03 : 0x01; // 11 for grayscale, 01 for 3-4 color
    int pixelsPerByte = 4;
    for (int col = 0; col < charWidth; col += pixelsPerByte) {
        uint8_t pixelByte = 0;
        for (int p = 0; p < pixelsPerByte; p++) {
            int pixelX = startX + charIdx * charWidth + col + p;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            uint8_t fontByte;
            int fontCol = (col + p) / fontScale;
            if (fontCol == 0 || fontCol > 7) {
                fontByte = 0x00;
            } else {
                fontByte = fontData[fontCol - 1];
            }
            uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
            uint8_t pixelValue = (pixelBit == 1) ? 0x00 : whiteCode;
            pixelByte |= (pixelValue << (6 - p * 2));
        }
        int bytePos = (startX + charIdx * charWidth + col) / 4;
        if (bytePos < pitch) {
            rowBuffer[bytePos] = pixelByte;
        }
    }
}

static void renderChar_1BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, int fontScale) {
    for (int col = 0; col < charWidth; col += fontScale) {
        uint8_t fontByte;
        int fontCol = col / fontScale;
        if (fontCol == 0 || fontCol > 7) {
            fontByte = 0x00;
        } else {
            fontByte = fontData[fontCol - 1];
        }
        uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
        for (int s = 0; s < fontScale; s++) {
            int pixelX = startX + charIdx * charWidth + col + s;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            
            int bytePos = pixelX / 8;
            int bitPos = 7 - (pixelX % 8);
            if (bytePos < pitch) {
                if (pixelBit == 1) {
                    rowBuffer[bytePos] &= ~(1 << bitPos);
                }
            }
        }
    }
}

void writeTextAndFill(const char* text) {
    if (text == nullptr || globalConfig.displays[0].pixel_width == 0) return;
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    bool useBitplanes = (colorScheme == 1 || colorScheme == 2); // BWR/BWY use bitplanes (grayscale uses 2BPP, not bitplanes)
    int bitsPerPixel = getBitsPerPixel();
    int pitch;
    uint8_t whiteValue;
    if (bitsPerPixel == 4) {
        pitch = globalConfig.displays[0].pixel_width / 2;
        whiteValue = 0xFF;
    } else if (bitsPerPixel == 2) {
        pitch = (globalConfig.displays[0].pixel_width + 3) / 4;
        if (colorScheme == 5) {
            whiteValue = 0xFF; // All pixels = 11 (white) for grayscale
        } else {
            whiteValue = 0x55; // All pixels = 01 (white) for 3-4 color displays
        }
    } else {
        pitch = (globalConfig.displays[0].pixel_width + 7) / 8;
        whiteValue = 0xFF;
    }
    int fontScale = (globalConfig.displays[0].pixel_width < FONT_SMALL_THRESHOLD) ? 1 : 2;
    int charWidth = FONT_BASE_WIDTH * fontScale;
    int charHeight = FONT_BASE_HEIGHT * fontScale;
    uint8_t* whiteRow = staticWhiteRow;
    memset(whiteRow, whiteValue, pitch);
    uint8_t* rowBuffer = staticRowBuffer;
    bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
    bbepStartWrite(&bbep, getplane());
    int maxChars = globalConfig.displays[0].pixel_width / charWidth;
    int lineCount = 1; // At least one line
    const char* countPtr = text;
    while (*countPtr != '\0') {
        if (*countPtr == '\n' || *countPtr == '\r') {
            lineCount++;
            if (*countPtr == '\r' && *(countPtr + 1) == '\n') {
                countPtr++; // Skip \r\n
            }
        }
        countPtr++;
    }
    int totalTextHeight = lineCount * charHeight;
    int remainingHeight = globalConfig.displays[0].pixel_height - totalTextHeight;
    int spacing = 0;
    if (lineCount > 0) {
        spacing = remainingHeight / (lineCount + 1);
    }
    const char* lineStart = text;
    const char* current = text;
    int currentY = 0;
    int lineIndex = 0;
    for (int s = 0; s < spacing && currentY < globalConfig.displays[0].pixel_height; s++) {
        bbepWriteData(&bbep, whiteRow, pitch);
        currentY++;
    }
    while (*current != '\0') {
        if (*current == '\n' || *current == '\r') {
            int lineLen = current - lineStart;
            if (lineLen > 0) {
                char* line = staticLineBuffer;
                int copyLen = (lineLen < 255) ? lineLen : 255;
                memcpy(line, lineStart, copyLen);
                line[copyLen] = '\0';
                int textLen = strlen(line);
                if (textLen > maxChars) textLen = maxChars;
                int textWidthPixels = textLen * charWidth;
                int startX = (globalConfig.displays[0].pixel_width - textWidthPixels) / 2;
                if (startX < 0) startX = 0;
                if (lineIndex > 0) {
                    for (int s = 0; s < spacing && currentY < globalConfig.displays[0].pixel_height; s++) {
                        bbepWriteData(&bbep, whiteRow, pitch);
                        currentY++;
                    }
                }
                for (int row = 0; row < charHeight && currentY < globalConfig.displays[0].pixel_height; row++) {
                    memset(rowBuffer, whiteValue, pitch);
                    int fontRow = row / fontScale;
                    for (int charIdx = 0; charIdx < textLen; charIdx++) {
                        uint8_t c = (uint8_t)line[charIdx];
                        if (c < 32 || c > 127) c = 32;
                        uint8_t fontData[7];
                        int fontOffset = (c - 32) * 7;
                        memcpy_P(fontData, &writelineFont[fontOffset], 7);
                        if (bitsPerPixel == 4) {
                            renderChar_4BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, fontScale);
                        } else if (bitsPerPixel == 2) {
                            renderChar_2BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, colorScheme, fontScale);
                        } else {
                            renderChar_1BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, fontScale);
                        }
                    }
                    bbepWriteData(&bbep, rowBuffer, pitch);
                    currentY++;
                }
                lineIndex++;
            } else {
                if (lineIndex > 0) {
                    for (int s = 0; s < spacing && currentY < globalConfig.displays[0].pixel_height; s++) {
                        bbepWriteData(&bbep, whiteRow, pitch);
                        currentY++;
                    }
                }
                for (int row = 0; row < charHeight && currentY < globalConfig.displays[0].pixel_height; row++) {
                    bbepWriteData(&bbep, whiteRow, pitch);
                    currentY++;
                }
                lineIndex++;
            }
            if (*current == '\r' && *(current + 1) == '\n') {
                current++;
            }
            current++;
            lineStart = current;
        } else {
            current++;
        }
    }
    if (current > lineStart) {
        int lineLen = current - lineStart;
        char* line = staticLineBuffer;
        int copyLen = (lineLen < 255) ? lineLen : 255;
        memcpy(line, lineStart, copyLen);
        line[copyLen] = '\0';
        if (lineIndex > 0) {
            for (int s = 0; s < spacing && currentY < globalConfig.displays[0].pixel_height; s++) {
                bbepWriteData(&bbep, whiteRow, pitch);
                currentY++;
            }
        }
        int textLen = strlen(line);
        if (textLen > maxChars) textLen = maxChars;
        int textWidthPixels = textLen * charWidth;
        int startX = (globalConfig.displays[0].pixel_width - textWidthPixels) / 2;
        if (startX < 0) startX = 0;
        
        for (int row = 0; row < charHeight && currentY < globalConfig.displays[0].pixel_height; row++) {
            memset(rowBuffer, whiteValue, pitch);
            int fontRow = row / fontScale;
            for (int charIdx = 0; charIdx < textLen; charIdx++) {
                uint8_t c = (uint8_t)line[charIdx];
                if (c < 32 || c > 127) c = 32;
                uint8_t fontData[7];
                int fontOffset = (c - 32) * 7;
                memcpy_P(fontData, &writelineFont[fontOffset], 7);
                if (bitsPerPixel == 4) {
                    renderChar_4BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, fontScale);
                } else if (bitsPerPixel == 2) {
                    renderChar_2BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, colorScheme, fontScale);
                } else {
                    renderChar_1BPP(rowBuffer, fontData, fontRow, charIdx, startX, charWidth, pitch, fontScale);
                }
            }
            
            bbepWriteData(&bbep, rowBuffer, pitch);
            currentY++;
        }
    }
    for (int s = 0; s < spacing && currentY < globalConfig.displays[0].pixel_height; s++) {
        bbepWriteData(&bbep, whiteRow, pitch);
        currentY++;
    }
    while (currentY < globalConfig.displays[0].pixel_height) {
        bbepWriteData(&bbep, whiteRow, pitch);
        currentY++;
    }
    if (currentY != globalConfig.displays[0].pixel_height) {
        writeSerial("WARNING: writeTextAndFill wrote " + String(currentY) + " rows, expected " + String(globalConfig.displays[0].pixel_height));
    } else {
        writeSerial("writeTextAndFill: Wrote " + String(currentY) + " rows (" + String(pitch) + " bytes/row, " + String(pitch * currentY) + " total bytes)");
    }
    if (useBitplanes) {
        memset(whiteRow, 0x00, pitch); // Reuse whiteRow, set to zeros = no red/yellow
        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
        bbepStartWrite(&bbep, PLANE_1);
        for (int row = 0; row < globalConfig.displays[0].pixel_height; row++) {
            bbepWriteData(&bbep, whiteRow, pitch);
        }
        writeSerial("writeTextAndFill: Wrote plane 2 (R/Y) with zeros for bitplane display");
    }
    if (colorScheme == 5) {
        int otherPlane = (getplane() == PLANE_0) ? PLANE_1 : PLANE_0;
        int otherPlanePitch = (globalConfig.displays[0].pixel_width + 7) / 8; // 1BPP pitch for the other plane
        memset(whiteRow, 0x00, otherPlanePitch); // Reuse whiteRow, set to zeros
        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
        bbepStartWrite(&bbep, otherPlane);
        for (int row = 0; row < globalConfig.displays[0].pixel_height; row++) {
            bbepWriteData(&bbep, whiteRow, otherPlanePitch);
        }
        writeSerial("writeTextAndFill: Cleared other plane (PLANE_" + String(otherPlane == PLANE_0 ? "0" : "1") + ") with zeros for grayscale display");
    }
}

void handleDirectWriteData(uint8_t* data, uint16_t len) {
    if (!directWriteActive) {
        writeSerial("ERROR: Direct write data received but mode not active");
        return;
    }
    if (len == 0) {
        writeSerial("WARNING: Empty data packet received");
        return;
    }
    if (directWriteCompressed) {
        handleDirectWriteCompressedData(data, len);
    } else {
        if (directWriteBitplanes) {
            uint16_t dataOffset = 0;
            uint16_t remainingLen = len;
            while (remainingLen > 0) {
                if (!directWritePlane2) {
                    uint32_t remainingInPlane = directWriteTotalBytes - directWriteBytesWritten;
                    uint16_t bytesToWrite = (remainingLen > remainingInPlane) ? remainingInPlane : remainingLen;
                    if (bytesToWrite > 0) {
                        bbepWriteData(&bbep, data + dataOffset, bytesToWrite);
                        directWriteBytesWritten += bytesToWrite;
                        writeSerial("Direct write plane 1: " + String(bytesToWrite) + " bytes written (total: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes) + ")");
                        dataOffset += bytesToWrite;
                        remainingLen -= bytesToWrite;
                    }
                    if (directWriteBytesWritten >= directWriteTotalBytes && !directWritePlane2) {
                        writeSerial("Plane 1 complete, switching to plane 2");
                        directWritePlane2 = true;
                        directWriteBytesWritten = 0;
                        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
                        bbepStartWrite(&bbep, PLANE_1);
                    }
                } else {
                    uint32_t remainingInPlane = directWriteTotalBytes - directWriteBytesWritten;
                    uint16_t bytesToWrite = (remainingLen > remainingInPlane) ? remainingInPlane : remainingLen;
                    if (bytesToWrite > 0) {
                        bbepWriteData(&bbep, data + dataOffset, bytesToWrite);
                        directWriteBytesWritten += bytesToWrite;
                        writeSerial("Direct write plane 2: " + String(bytesToWrite) + " bytes written (total: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes) + ")");
                        dataOffset += bytesToWrite;
                        remainingLen -= bytesToWrite;
                    }
                    if (directWriteBytesWritten >= directWriteTotalBytes) {
                        writeSerial("Plane 2 complete");
                        break;
                    }
                }
            }
            if (directWritePlane2 && directWriteBytesWritten >= directWriteTotalBytes) {
                writeSerial("All planes written, ending direct write mode");
                handleDirectWriteEnd();
            } else {
                uint8_t ackResponse[] = {0x00, 0x71};
                sendResponse(ackResponse, sizeof(ackResponse));
            }
        } else {
            bbepWriteData(&bbep, data, len);
            directWriteBytesWritten += len;
            writeSerial("Direct write: " + String(len) + " bytes written (total: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes) + ")");
            if (directWriteBytesWritten >= directWriteTotalBytes) {
                writeSerial("All data written, ending direct write mode");
                handleDirectWriteEnd();
            } else {
                uint8_t ackResponse[] = {0x00, 0x71};
                sendResponse(ackResponse, sizeof(ackResponse));
            }
        }
    }
}

void handleDirectWriteCompressedData(uint8_t* data, uint16_t len) {
    uint32_t newTotalSize = directWriteCompressedReceived + len;
    if (newTotalSize > MAX_IMAGE_SIZE) {
        writeSerial("ERROR: Compressed data exceeds static buffer size (" + String(newTotalSize) + " > " + String(MAX_IMAGE_SIZE) + ")");
        writeSerial("Rejecting compressed upload - client should use uncompressed mode");
        directWriteActive = false;
        directWriteCompressed = false;
        directWriteCompressedReceived = 0;
        directWriteCompressedBuffer = nullptr;
        uint8_t errorResponse[] = {0xFF, 0xFF};  // Error response: use uncompressed upload
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    memcpy(directWriteCompressedBuffer + directWriteCompressedReceived, data, len);
    directWriteCompressedReceived += len;
    writeSerial("Accumulated compressed data: " + String(directWriteCompressedReceived) + " bytes");
    uint8_t ackResponse[] = {0x00, 0x71};
    sendResponse(ackResponse, sizeof(ackResponse));
}

void decompressDirectWriteData() {
    if (directWriteCompressedReceived == 0) {
        writeSerial("ERROR: No compressed data to decompress");
        return;
    }
    writeSerial("Starting decompression of " + String(directWriteCompressedReceived) + " bytes compressed data");
    struct uzlib_uncomp d;
    memset(&d, 0, sizeof(d));
    d.source = directWriteCompressedBuffer;
    d.source_limit = directWriteCompressedBuffer + directWriteCompressedReceived;
    d.source_read_cb = NULL;
    uzlib_init();
    int hdr = uzlib_zlib_parse_header(&d);
    if (hdr < 0) {
        writeSerial("ERROR: Invalid zlib header: " + String(hdr));
        return;
    }
    uint16_t window = 0x100 << hdr;
    if (window > MAX_DICT_SIZE) window = MAX_DICT_SIZE;
    uzlib_uncompress_init(&d, dictionaryBuffer, window);
    if (directWriteBitplanes) {
        writeSerial("Bitplane mode: streaming decompression");
        bool decompressingPlane2 = false;
        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
        bbepStartWrite(&bbep, PLANE_0);
        directWriteBytesWritten = 0;
        int res;
        do {
            d.dest_start = decompressionChunk;
            d.dest = decompressionChunk;
            d.dest_limit = decompressionChunk + DECOMP_CHUNK_SIZE;
            res = uzlib_uncompress(&d);
            size_t bytesOut = d.dest - d.dest_start;
            if (bytesOut > 0) {
                uint32_t remainingInPlane1 = directWriteTotalBytes - directWriteBytesWritten;
                if (bytesOut > remainingInPlane1) {
                    bbepWriteData(&bbep, decompressionChunk, remainingInPlane1);
                    directWriteBytesWritten = directWriteTotalBytes;
                    writeSerial("Plane 1 complete: " + String(directWriteTotalBytes) + " bytes");
                    decompressingPlane2 = true;
                    bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
                    bbepStartWrite(&bbep, PLANE_1);
                    directWriteBytesWritten = 0;
                    uint32_t plane2Bytes = bytesOut - remainingInPlane1;
                    bbepWriteData(&bbep, decompressionChunk + remainingInPlane1, plane2Bytes);
                    directWriteBytesWritten = plane2Bytes;
                    writeSerial("Plane 2 started: " + String(plane2Bytes) + " bytes");
                } else {
                    bbepWriteData(&bbep, decompressionChunk, bytesOut);
                    directWriteBytesWritten += bytesOut;
                    if (directWriteBytesWritten >= directWriteTotalBytes) {
                        writeSerial("Plane 1 complete: " + String(directWriteTotalBytes) + " bytes");
                        decompressingPlane2 = true;
                        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
                        bbepStartWrite(&bbep, PLANE_1);
                        directWriteBytesWritten = 0;
                        writeSerial("Switched to plane 2");
                    }
                }
            }
            if (res == TINF_DATA_ERROR) {
                writeSerial("ERROR: Decompression data error");
                break;
            }
        } while (res == TINF_OK && !decompressingPlane2);
        if (decompressingPlane2 && res == TINF_OK) {
            do {
                d.dest_start = decompressionChunk;
                d.dest = decompressionChunk;
                d.dest_limit = decompressionChunk + DECOMP_CHUNK_SIZE;
                res = uzlib_uncompress(&d);
                size_t bytesOut = d.dest - d.dest_start;
                if (bytesOut > 0) {
                    uint32_t remainingInPlane2 = directWriteTotalBytes - directWriteBytesWritten;
                    uint32_t bytesToWrite = (bytesOut > remainingInPlane2) ? remainingInPlane2 : bytesOut;
                    bbepWriteData(&bbep, decompressionChunk, bytesToWrite);
                    directWriteBytesWritten += bytesToWrite;
                    writeSerial("Plane 2: " + String(bytesToWrite) + " bytes, total: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes));
                }
                if (res == TINF_DATA_ERROR) {
                    writeSerial("ERROR: Decompression data error");
                    break;
                }
            } while (res == TINF_OK && directWriteBytesWritten < directWriteTotalBytes);
        }
        if (res == TINF_DONE || (decompressingPlane2 && directWriteBytesWritten >= directWriteTotalBytes)) {
            writeSerial("Bitplane decompression complete: plane 1 + plane 2 = " + String(directWriteTotalBytes * 2) + " bytes");
        } else {
            writeSerial("ERROR: Decompression failed with code: " + String(res));
        }
    } else {
        int res;
        do {
            d.dest_start = decompressionChunk;
            d.dest = decompressionChunk;
            d.dest_limit = decompressionChunk + DECOMP_CHUNK_SIZE;
            res = uzlib_uncompress(&d);
            size_t bytesOut = d.dest - d.dest_start;
            if (bytesOut > 0) {
                bbepWriteData(&bbep, decompressionChunk, bytesOut);
                directWriteBytesWritten += bytesOut;
                writeSerial("Decompressed: " + String(bytesOut) + " bytes, written: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes));
            }
            if (res == TINF_DATA_ERROR) {
                writeSerial("ERROR: Decompression data error");
                break;
            }
        } while (res == TINF_OK && directWriteBytesWritten < directWriteTotalBytes);
        
        if (res == TINF_DONE || directWriteBytesWritten >= directWriteTotalBytes) {
            writeSerial("Decompression complete: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes) + " bytes written");
        } else {
            writeSerial("ERROR: Decompression failed with code: " + String(res));
        }
    }
}

void handleDirectWriteEnd() {
    if (!directWriteActive) {
        writeSerial("WARNING: Direct write end called but mode not active");
        return;
    }
    writeSerial("=== DIRECT WRITE END ===");
    if (directWriteCompressed && directWriteCompressedReceived > 0) {
        writeSerial("Decompressing accumulated compressed data...");
        decompressDirectWriteData();
    }
    if (directWriteBitplanes) {
        writeSerial("Total bytes written: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes) + " per plane (" + String(directWritePlane2 ? "plane 2" : "plane 1") + ")");
    } else {
        writeSerial("Total bytes written: " + String(directWriteBytesWritten) + "/" + String(directWriteTotalBytes));
    }
    if (directWriteCompressed) {
        writeSerial("Compressed bytes received: " + String(directWriteCompressedReceived));
    }
    bbepRefresh(&bbep, REFRESH_FULL);
    waitforrefresh(60);
    pwrmgm(false);
    directWriteActive = false;
    directWriteCompressed = false;
    directWriteBytesWritten = 0;
    directWriteCompressedReceived = 0;
    directWriteCompressedSize = 0;
    directWriteDecompressedTotal = 0;
    directWriteCompressedBuffer = nullptr;
    directWriteWidth = 0;
    directWriteHeight = 0;
    directWriteTotalBytes = 0;
    uint8_t ackResponse[] = {0x00, 0x72};
    sendResponse(ackResponse, sizeof(ackResponse));
    writeSerial("Direct write completed and display refreshed");
}

void imageDataWritten(BLEConnHandle conn_hdl, BLECharPtr chr, uint8_t* data, uint16_t len) {
    if (len < 2) {
        writeSerial("ERROR: Command too short (" + String(len) + " bytes)");
        return;
    }
    uint16_t command = (data[0] << 8) | data[1];  // Fixed byte order
    writeSerial("Processing command: 0x" + String(command, HEX));
    switch (command) {
        case 0x0040: // Read Config command
            writeSerial("=== READ CONFIG COMMAND (0x0040) ===");
            writeSerial("Command received at time: " + String(millis()));
            handleReadConfig();
            writeSerial("Returned from handleReadConfig");
            break;
        case 0x0041: // Write Config command
            writeSerial("=== WRITE CONFIG COMMAND (0x0041) ===");
            handleWriteConfig(data + 2, len - 2);
            break;
        case 0x0042: // Write Config Chunk command
            writeSerial("=== WRITE CONFIG CHUNK COMMAND (0x0042) ===");
            handleWriteConfigChunk(data + 2, len - 2);
            break;
        case 0x000F: // Reboot
            writeSerial("=== Reboot COMMAND (0x000F) ===");
            delay(100);
            reboot();
            break;
        case 0x0043: // Firmware Version command
            writeSerial("=== FIRMWARE VERSION COMMAND (0x0043) ===");
            handleFirmwareVersion();
            break;
        case 0x0070: // Direct Write Start command
            writeSerial("=== DIRECT WRITE START COMMAND (0x0070) ===");
            handleDirectWriteStart(data + 2, len - 2);
            break;
        case 0x0071: // Direct Write Data command
            handleDirectWriteData(data + 2, len - 2);
            break;
        case 0x0072: // Direct Write End command
            writeSerial("=== DIRECT WRITE END COMMAND (0x0072) ===");
            handleDirectWriteEnd();
            break;
        default:
            writeSerial("ERROR: Unknown command: 0x" + String(command, HEX));
            writeSerial("Expected: 0x0011 (read config), 0x0064 (image info), 0x0065 (block data), or 0x0003 (finalize)");
            break;
    }
    writeSerial("Command processing completed successfully");
}