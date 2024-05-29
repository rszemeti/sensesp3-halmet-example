#include "system_config.h"
// Sensor configurations

std::vector<AnalogueSensorConfig> analogue_sensor_configs = {
    {TANK_LEVEL_SENSOR,  0, "A1", "/Main Engine Fuel", "Fuel Tank, Main Engine", nullptr, 0.0f, 1.0f, "l", "Tank Volume", "Calculated tank remaining volume", "Tank Volume"},
    {TEMPERATURE_SENSOR, 1, "A2", "/Main Engine Coolant Temperature", "Engine Coolant Temperature", nullptr, 0.0f, 0.0f, "C", "Engine Temperature", "Temperature", "Temp"},
    {PRESSURE_SENSOR,    2, "A3", "/Main Engine Oil Pressure", "Engine Oil Pressure",  nullptr, 25.0f, 0.0f, "Pa", "Oil Pressure", "Pressure", "Press"}
};

std::vector<TachoInputConfig> tacho_input_configs = {
    {1, "D1", "/Tacho D1", "Enable RPM input D1", "Hz", "Main Engine Revolutions", "Main Engine Revolutions", "Revolutions", 5000}
};

std::vector<DigitalInputConfig> digital_input_configs = {
    {2, "D2", LOW_OIL, false},
    {3, "D3", OVER_TEMP, true}
};

std::vector<OneWireTemperatureConfig> onewire_temperature_configs = {
    {"Oil Temperature", "/Temperature 1", "Engine oil temperature sensor on the 1-Wire bus.", "/Temperature 1/Oil Temperature Alarm", 383, "K", "Engine Oil Temperature", "Engine Oil Temperature", "Oil Temperature", 10000},
    {"Coolant Temperature", "/Temperature 2", "Engine coolant temperature sensor on the 1-Wire bus.", "/Temperature 2/Coolant Temperature Alarm", 373, "K", "Engine Coolant Temperature", "Engine Coolant Temperature", "Coolant Temperature", 11000},
    {"Exhaust Temperature", "/Temperature 3", "Engine wet exhaust temperature sensor on the 1-Wire bus.", "/Temperature 3/Exhaust Temperature Alarm", 333, "K", "Wet Exhaust Temperature", "Wet Exhaust Temperature", "Exhaust Temperature", 12000}
};