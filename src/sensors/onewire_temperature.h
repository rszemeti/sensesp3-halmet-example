#ifndef ONEWIRE_TEMPERATURE_SENSOR_H
#define ONEWIRE_TEMPERATURE_SENSOR_H

#include "config.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_SSD1306.h>
#include <sensesp/signalk/signalk_output.h>
#include <sensesp/system/lambda_consumer.h>
#include <sensesp/transforms/lambda_transform.h>
#include <sensesp/system/lambda_consumer.h>

#include "n2k_senders.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/ui/ui_controls.h"
#include "sensesp_onewire/onewire_temperature.h"

using namespace sensesp;

// OneWire temperature sensor configurations
struct OneWireTemperatureConfig {
    const char* input_name;
    const char* sk_path;
    const char* description;
    const char* alarm_path;
    float default_limit;
    const char* metadata_units;
    const char* metadata_display_name;
    const char* metadata_description;
    const char* metadata_short_name;
    int initial_sort_order;
};

class OneWireTemperatureSensor {
public:
    OneWireTemperatureSensor(const char* input_name, const char* sk_path, 
                             const char* description, const char* alarm_path, float default_limit, 
                             const char* metadata_units, const char* metadata_display_name, 
                             const char* metadata_description, const char* metadata_short_name, 
                             int initial_sort_order)
        : input_name_(input_name), sk_path_(sk_path), description_(description), 
          alarm_path_(alarm_path), default_limit_(default_limit), metadata_units_(metadata_units), 
          metadata_display_name_(metadata_display_name), metadata_description_(metadata_description), 
          metadata_short_name_(metadata_short_name), initial_sort_order_(initial_sort_order) {}

    void connect(DallasTemperatureSensors* dts, N2kEngineParameterDynamicSender* n2k_engine_dynamic_sender, bool enable_n2k_output) {
        String sk_path_onewire = String(sk_path_) + "/OneWire";
        auto* temperature_sensor = new OneWireTemperature(dts, 1000, sk_path_onewire);
        temperature_sensor->set_description(description_);
        temperature_sensor->set_sort_order(initial_sort_order_);

        auto temperature_sk_output = new SKOutput<float>(
            String(sk_path_) + "/Current Temperature", String(sk_path_) + "/SK Path",
            new SKMetadata(metadata_units_, metadata_display_name_, metadata_description_, metadata_short_name_));
        temperature_sk_output->set_sort_order(initial_sort_order_ + 100);
        temperature_sensor->connect_to(temperature_sk_output);

        if(alarm_path_ != nullptr) {
            auto alarm_function = [](float temperature, float limit) -> bool {
                return temperature > limit;
            };
            const ParamInfo* temperature_limit = new ParamInfo[1]{{"temperature_limit", "Temperature Limit"}};
            auto* temperature_alarm = new LambdaTransform<float, bool, float>(
                alarm_function, default_limit_, temperature_limit, String(alarm_path_) + "/Temperature Alarm");
            temperature_alarm->set_description("Alarm if the temperature exceeds the set limit. Value in " + String(metadata_units_) + ".");
            temperature_alarm->set_sort_order(initial_sort_order_ + 200);
            temperature_sensor->connect_to(temperature_alarm);
            if (enable_n2k_output) {
                temperature_alarm->connect_to(&(n2k_engine_dynamic_sender->over_temperature_consumer_));
            }
        }

        if (enable_n2k_output) {
            temperature_sensor->connect_to(&(n2k_engine_dynamic_sender->oil_temperature_consumer_));
        }
    }

private:
    const char* input_name_;
    const char* sk_path_;
    const char* description_;
    const char* alarm_path_;
    float default_limit_;
    const char* metadata_units_;
    const char* metadata_display_name_;
    const char* metadata_description_;
    const char* metadata_short_name_;
    int initial_sort_order_;
};

#endif // ONEWIRE_TEMPERATURE_SENSOR_H
