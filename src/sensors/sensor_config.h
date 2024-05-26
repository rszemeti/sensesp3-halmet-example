#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include "analogue_temperature.h"
#include "tank_level.h"



enum SensorType {
    TEMPERATURE_SENSOR,
    TANK_LEVEL_SENSOR,
    PRESSURE_SENSOR
};

struct AnalogueSensorConfig {
    SensorType type;
    int ads_channel;
    const char* input_name;
    const char* sk_path;
    const char* description;
    const char* alarm_path; // Only used for temperature sensors
    float default_limit;    // Only used for temperature sensors
    float tank_size;        // Only used for tank level sensors
    const char* metadata_units;
    const char* metadata_display_name;
    const char* metadata_description;
    const char* metadata_short_name;
    int initial_sort_order;
};

enum AlarmType {
    LOW_OIL,
    OVER_TEMP
};

// Digital input configurations
struct DigitalInputConfig {
    int pin;
    const char* input_name;
    AlarmType alarm_type;
    bool inverted;
};
#endif // SENSOR_CONFIG_H
