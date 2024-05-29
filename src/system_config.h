#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "sensors/analogue_sensor_config.h"
#include "sensors/tacho.h"
#include "sensors/digital_alarm.h"
#include "sensors/onewire_temperature.h"

// Sensor configurations
// Vectors to store configurations
extern std::vector<AnalogueSensorConfig> analogue_sensor_configs;
extern std::vector<TachoInputConfig> tacho_configs;  
extern std::vector<DigitalInputConfig> digital_input_configs;
extern std::vector<OneWireTemperatureConfig> onewire_temperature_configs;

#endif // SYSTEM_CONFIG_H
