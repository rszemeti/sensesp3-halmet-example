#ifndef ANALOG_TEMPERATURE_SENSOR_H
#define ANALOG_TEMPERATURE_SENSOR_H

#include "config.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_SSD1306.h>


#include "n2k_senders.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/ui/ui_controls.h"

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_display.h"
#include "n2k_senders.h"

using namespace sensesp;

class AnalogTemperatureSensor {
public:
    AnalogTemperatureSensor(int ads_channel, const char* input_name, const char* sk_path, 
                            const char* description, const char* alarm_path, float default_limit, const char* metadata_units, 
                            const char* metadata_display_name, const char* metadata_description, const char* metadata_short_name)
        : ads_channel_(ads_channel), input_name_(input_name), sk_path_(sk_path), description_(description), 
          alarm_path_(alarm_path), default_limit_(default_limit), metadata_units_(metadata_units), 
          metadata_display_name_(metadata_display_name), metadata_description_(metadata_description), 
          metadata_short_name_(metadata_short_name) {
            initial_sort_order_ = ads_channel_ * 1000 + 2000;
          }

    void connect(Adafruit_ADS1115* ads, N2kEngineParameterDynamicSender* n2k_engine_dynamic_sender, bool enable_n2k_output, bool display_present, Adafruit_SSD1306* display) {
        String sk_path_enabled = String(sk_path_) + "/Enabled";
        auto* enable_input = new CheckboxConfig(true, "Enable Input", sk_path_enabled);
        enable_input->set_description(String("Enable analog input ") + input_name_ + ". Requires a reboot to take effect.");
        enable_input->set_sort_order(initial_sort_order_);

        if (enable_input->get_value()) {
            // Connect the analog sender
            auto analog_resistance = ConnectAnalogSender(ads, ads_channel_, input_name_);

            // Resistance converted to temperature
            auto temperature_curve = new CurveInterpolator(nullptr, String(sk_path_) + "/Temperature Curve");
            temperature_curve->set_input_title("Sender Resistance (ohms)")
                              ->set_output_title("Temperature (C)")
                              ->set_description("Piecewise linear conversion of the resistance to temperature")
                              ->set_sort_order(initial_sort_order_ + 100);
            if (temperature_curve->get_samples().empty()) {
                // Provide a default curve if there's no prior configuration
                temperature_curve->clear_samples();
                temperature_curve->add_sample(CurveInterpolator::Sample(0, 0));
                temperature_curve->add_sample(CurveInterpolator::Sample(180., 300.));
            }
            analog_resistance->connect_to(temperature_curve);

            // Signal K output
            auto temperature_sk_output = new SKOutputFloat(
                String(sk_path_) + "/Current Temperature", String(sk_path_) + "/SK Path",
                new SKMetadata(metadata_units_, metadata_display_name_, metadata_description_, metadata_short_name_));
            temperature_sk_output->set_sort_order(initial_sort_order_ + 200);
            temperature_curve->connect_to(temperature_sk_output);

            // Alarm setup
            if( alarm_path_ != nullptr) {
                // Alarm setup (only for temperature sensors)
                // Connect the temperature output to the alarm function (temperature > limit)
                const ParamInfo* temperature_limit = new ParamInfo[1]{{"temperature_limit", "Temperature Limit"}};
                auto alarm_function = [](float temperature, float limit) -> bool {
                    return temperature > limit;
                };
                auto* temperature_alarm = new LambdaTransform<float, bool, float>(
                    alarm_function, default_limit_, temperature_limit, String(alarm_path_) + "/Temperature Alarm");
                temperature_alarm->set_description("Alarm if the temperature exceeds the set limit. Value in " + String(metadata_units_) + ".");
                temperature_alarm->set_sort_order(initial_sort_order_ + 300);
                temperature_curve->connect_to(temperature_alarm);
                if(enable_n2k_output){
                    temperature_alarm->connect_to(&(n2k_engine_dynamic_sender->over_temperature_consumer_));
                }
            }

#ifdef ENABLE_SIGNALK
            auto analog_resistance_sk_output = new SKOutputFloat(
                String(sk_path_) + "/Sender Resistance", String(sk_path_) + "/Sender Resistance",
                new SKMetadata("ohm", "Input sender resistance", "Input sender resistance"));
            analog_resistance_sk_output->set_sort_order(initial_sort_order_ + 400);
            analog_resistance->connect_to(analog_resistance_sk_output);
#endif

            if (enable_n2k_output) {
                // Connect the temperature output to N2k dynamic sender
                auto kelvin_function = [](float temperature) -> float {
                    return temperature+273.15;
                };
                auto* temperature_kelvin = new LambdaTransform<float, float>(
                    kelvin_function, "Temperature Kelvin"); 
                temperature_curve->connect_to(temperature_kelvin);
                
                temperature_kelvin->connect_to(&(n2k_engine_dynamic_sender->temperature_consumer_));
            }

            if (display_present && display != nullptr) {
                temperature_curve->connect_to(new LambdaConsumer<float>(
                    [this, display](float value) { PrintValue(display, ads_channel_ + 2 , "Temperature", value); }));
            }
        }
    }

private:
    int ads_channel_;
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
#endif // ANALOG_TEMPERATURE_SENSOR_H
