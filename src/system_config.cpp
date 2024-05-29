#include "system_config.h"
#include <FS.h>
#include <SPIFFS.h>

SystemConfig::SystemConfig() {
    // Optionally, you can initialize with default values
}

const std::vector<AnalogueSensorConfig>& SystemConfig::getAnalogueSensorConfigs() const {
    return analogue_sensor_configs_;
}

const std::vector<TachoInputConfig>& SystemConfig::getTachoInputConfigs() const {
    return tacho_input_configs_;
}

const std::vector<DigitalInputConfig>& SystemConfig::getDigitalInputConfigs() const {
    return digital_input_configs_;
}

const std::vector<OneWireTemperatureConfig>& SystemConfig::getOneWireTemperatureConfigs() const {
    return onewire_temperature_configs_;
}

bool SystemConfig::loadFromJson(const char* filename) {
    // Initialize SPIFFS
    if (!SPIFFS.begin()) {
        Serial.println("Failed to mount file system");
        return false;
    }

    // Open the JSON file
    File configFile = SPIFFS.open(filename, "r");
    if (!configFile) {
        Serial.println("Failed to open config file");
        return false;
    }

    // Parse the JSON file
    size_t size = configFile.size();
    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);

    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, buf.get());
    if (error) {
        Serial.println("Failed to parse config file");
        return false;
    }

    // Load analog sensors
    JsonArray analogueSensors = doc["analogueSensors"].as<JsonArray>();
    for (JsonObject sensor : analogueSensors) {
        AnalogueSensorConfig config;
        config.type = sensor["type"];
        config.ads_channel = sensor["ads_channel"];
        config.input_name = sensor["input_name"];
        config.sk_path = sensor["sk_path"];
        config.alarm_path = sensor["alarm_path"];
        config.description = sensor["description"];
        config.default_limit = sensor["default_limit"];
        config.metadata_units = sensor["metadata_units"];
        config.metadata_display_name = sensor["metadata_display_name"];
        config.metadata_description = sensor["metadata_description"];
        config.metadata_short_name = sensor["metadata_short_name"];
        analogue_sensor_configs_.push_back(config);
    }

    // Load tacho inputs
    JsonArray tachoInputs = doc["tachoInputs"].as<JsonArray>();
    for (JsonObject sensor : tachoInputs) {
        TachoInputConfig config;
        config.pin = sensor["pin"];
        config.input_name = sensor["input_name"];
        config.sk_path = sensor["sk_path"];
        config.description = sensor["description"];
        config.metadata_units = sensor["metadata_units"];
        config.metadata_display_name = sensor["metadata_display_name"];
        config.metadata_description = sensor["metadata_description"];
        config.metadata_short_name = sensor["metadata_short_name"];
        config.initial_sort_order = sensor["initial_sort_order"];
        tacho_input_configs_.push_back(config);
    }

    // Load digital inputs
    JsonArray digitalInputs = doc["digitalInputs"].as<JsonArray>();
    for (JsonObject sensor : digitalInputs) {
        DigitalInputConfig config;
        config.pin = sensor["pin"];
        config.input_name = sensor["input_name"];
        config.alarm_type = sensor["alarm_type"];
        config.inverted = sensor["inverted"];
        digital_input_configs_.push_back(config);
    }

    // Load OneWire temperature sensors
    JsonArray oneWireTemperatures = doc["oneWireTemperatures"].as<JsonArray>();
    for (JsonObject sensor : oneWireTemperatures) {
        OneWireTemperatureConfig config;
        config.input_name = sensor["input_name"];
        config.sk_path = sensor["sk_path"];
        config.description = sensor["description"];
        config.alarm_path = sensor["alarm_path"];
        config.default_limit = sensor["default_limit"];
        config.metadata_units = sensor["metadata_units"];
        config.metadata_display_name = sensor["metadata_display_name"];
        config.metadata_description = sensor["metadata_description"];
        config.metadata_short_name = sensor["metadata_short_name"];
        config.initial_sort_order = sensor["initial_sort_order"];
        onewire_temperature_configs_.push_back(config);
    }

    return true;
}
