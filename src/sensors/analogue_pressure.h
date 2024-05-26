#ifndef ANALOG_PRESSURE_SENSOR_H
#define ANALOG_PRESSURE_SENSOR_H

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


class AnalogPressureSensor {
public:
    AnalogPressureSensor( int ads_channel, const char* input_name, const char* sk_path, 
                         const char* description, const char* alarm_path, float default_limit, const char* metadata_units, 
                         const char* metadata_display_name, const char* metadata_description, const char* metadata_short_name 
                        )
        : ads_channel_(ads_channel), input_name_(input_name), sk_path_(sk_path), description_(description), 
          alarm_path_(alarm_path), default_limit_(default_limit), metadata_units_(metadata_units), 
          metadata_display_name_(metadata_display_name), metadata_description_(metadata_description), 
          metadata_short_name_(metadata_short_name){
            initial_sort_order_ = ads_channel_ * 1000 +2000;
          }

    void connect(Adafruit_ADS1115* ads, N2kEngineParameterDynamicSender* n2k_engine_dynamic_sender, bool enable_n2k_output, bool display_present, Adafruit_SSD1306* display) {
        String sk_path_enabled = String(sk_path_) + "/Enabled";
        auto* enable_input = new CheckboxConfig(true, "Enable Input", sk_path_enabled);
        enable_input->set_description(String("Enable analog input ") + input_name_ + ". Requires a reboot to take effect.");
        enable_input->set_sort_order(initial_sort_order_);

        if (enable_input->get_value()) {
            // Connect the analog sender
            auto analog_resistance = ConnectAnalogSender(ads, ads_channel_, input_name_);

            // Resistance converted to pressure
            auto pressure_curve = new CurveInterpolator(nullptr, String(sk_path_) + "/Pressure Curve");
            pressure_curve->set_input_title("Sender Resistance (ohms)")
                          ->set_output_title("Pressure (Pa)")
                          ->set_description("Piecewise linear conversion of the resistance to pressure")
                          ->set_sort_order(initial_sort_order_ + 100);
            if (pressure_curve->get_samples().empty()) {
                // Provide a default curve if there's no prior configuration
                pressure_curve->clear_samples();
                pressure_curve->add_sample(CurveInterpolator::Sample(0, 0));
                pressure_curve->add_sample(CurveInterpolator::Sample(180., 300.));
            }
            analog_resistance->connect_to(pressure_curve);

            // Signal K output
            auto pressure_sk_output = new SKOutputFloat(
                String(sk_path_) + "/Current Pressure", String(sk_path_) + "/SK Path",
                new SKMetadata(metadata_units_, metadata_display_name_, metadata_description_, metadata_short_name_));
            pressure_sk_output->set_sort_order(initial_sort_order_ + 200);
            pressure_curve->connect_to(pressure_sk_output);

            // Alarm setup
            const ParamInfo* pressure_limit = new ParamInfo[1]{{"pressure_limit", "Pressure Limit"}};
            auto alarm_function = [](float pressure, float limit) -> bool {
                return pressure < limit;
            };
            auto* pressure_alarm = new LambdaTransform<float, bool, float>(
                alarm_function, default_limit_, pressure_limit, String(alarm_path_) + "/Pressure Alarm");
            pressure_alarm->set_description("Alarm if the pressure falls below the set limit. Value in " + String(metadata_units_) + ".");
            pressure_alarm->set_sort_order(initial_sort_order_ + 300);
            pressure_curve->connect_to(pressure_alarm);

#ifdef ENABLE_SIGNALK
            auto analog_resistance_sk_output = new SKOutputFloat(
                String(sk_path_) + "/Sender Resistance", String(sk_path_) + "/Sender Resistance",
                new SKMetadata("ohm", "Input sender resistance", "Input sender resistance"));
            analog_resistance_sk_output->set_sort_order(initial_sort_order_ + 400);
            analog_resistance->connect_to(analog_resistance_sk_output);
#endif

            if (enable_n2k_output) {
                // Connect the pressure output to N2k dynamic sender
                pressure_curve->connect_to(&(n2k_engine_dynamic_sender->oil_pressure_consumer_));

                // Connect the pressure alarm to N2k dynamic sender
                pressure_alarm->connect_to(&(n2k_engine_dynamic_sender->low_oil_pressure_consumer_));
            }

            if (display_present && display != nullptr) {
                pressure_curve->connect_to(new LambdaConsumer<float>(
                    [this, display](float value) { PrintValue(display, 2, String("Pressure ") + input_name_, value); }));
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

#endif // ANALOG_PRESSURE_SENSOR_H
