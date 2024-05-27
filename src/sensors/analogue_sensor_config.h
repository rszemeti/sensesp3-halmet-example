#ifndef ANALOGUE_SENSOR_CONFIG_H
#define ANALOGUE_SENSOR_CONFIG_H

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
    const char* alarm_path; // Only used for temperature and pressure sensors, set nulptr for no alarm
    float default_limit;    // Only used for temperature and pressure sensors
    float tank_size;        // Only used for tank level sensors
    const char* metadata_units;
    const char* metadata_display_name;
    const char* metadata_description;
    const char* metadata_short_name;
    int initial_sort_order;
};

#endif // ANALOGUE_SENSOR_CONFIG_H
