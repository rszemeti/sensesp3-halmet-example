#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <vector>
#include "sensors/analogue_sensor_config.h"
#include "sensors/tacho.h"
#include "sensors/digital_alarm.h"
#include "sensors/onewire_temperature.h"
#include <ArduinoJson.h>

class SystemConfig {
public:
    SystemConfig();
    
    const std::vector<AnalogueSensorConfig>& getAnalogueSensorConfigs() const;
    const std::vector<TachoInputConfig>& getTachoInputConfigs() const;
    const std::vector<DigitalInputConfig>& getDigitalInputConfigs() const;
    const std::vector<OneWireTemperatureConfig>& getOneWireTemperatureConfigs() const;

    bool loadFromJson(const char* filename);

private:
    std::vector<AnalogueSensorConfig> analogue_sensor_configs_;
    std::vector<TachoInputConfig> tacho_input_configs_;
    std::vector<DigitalInputConfig> digital_input_configs_;
    std::vector<OneWireTemperatureConfig> onewire_temperature_configs_;
};

#endif // SYSTEM_CONFIG_H
