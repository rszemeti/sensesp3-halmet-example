#ifndef DIGITAL_TEMPERATURE_SENSOR_H
#define DIGITAL_TEMPERATURE_SENSOR_H

#include "config.h"  // Make sure to include the necessary headers
#include "sensesp_onewire/onewire_temperature.h"

template<const char* SENSOR_NAME, const char* SK_PATH, const char* DESCRIPTION, const char* ALARM_PATH, int INITIAL_SORT_ORDER>
void setupDigitalTemperatureSensor(sensesp::DallasTemperatureSensors* dts, float default_limit, const char* metadata_units, const char* metadata_display_name, const char* metadata_description, const char* metadata_short_name) {
  auto* enable_input =
      new CheckboxConfig(true, "Enable Temperature Sensor", std::string(SK_PATH) + "/Enabled");
  enable_input->set_description(
      "Enable " + std::string(SENSOR_NAME) + ". Requires a reboot to take effect.");
  enable_input->set_sort_order(INITIAL_SORT_ORDER);

  if (enable_input->get_value()) {
    // Metadata
    auto* metadata = new SKMetadata(metadata_units, metadata_display_name, metadata_description, metadata_short_name, 10.0);

    // OneWireTemperature sensor
    auto* temperature_sensor = new OneWireTemperature(dts, 1000, std::string(SK_PATH) + "/OneWire");
    temperature_sensor->set_description(DESCRIPTION);
    temperature_sensor->set_sort_order(INITIAL_SORT_ORDER + 100);

    // SKOutput
    auto* temperature_sk_output = new SKOutput<float>(
        std::string(SK_PATH) + "/Temperature", std::string(SK_PATH) + "/SK Path", metadata);
    temperature_sk_output->set_sort_order(INITIAL_SORT_ORDER + 200);
    temperature_sensor->connect_to(temperature_sk_output);

    // Alarm
    const ParamInfo* temperature_limit = new ParamInfo[1]{{"temperature_limit", "Temperature Limit"}};
    auto temperature_alarm_function = [](float temperature, float limit) -> bool {
      return temperature > limit;
    };
    auto* temperature_alarm = new LambdaTransform<float, bool, float>(
        temperature_alarm_function,
        default_limit,  // Default value for parameter
        temperature_limit,  // Parameter UI description
        std::string(ALARM_PATH) + "/Temperature Alarm");
    temperature_alarm->set_description("Alarm if the temperature exceeds the set limit. Value in " + std::string(metadata_units) + ".");
    temperature_alarm->set_sort_order(INITIAL_SORT_ORDER + 300);
    temperature_sensor->connect_to(temperature_alarm);

    if (enable_n2k_output->get_value()) {
      // Connect the temperature output to N2k dynamic sender
      temperature_sensor->connect_to(&(n2k_engine_dynamic_sender->temperature_consumer_));
      // Connect the temperature alarm to N2k dynamic sender
      temperature_alarm->connect_to(temp_alarm, 0);
    }
  }
}

#endif // DIGITAL_TEMPERATURE_SENSOR_H
