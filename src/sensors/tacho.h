#ifndef TACHO_INPUT_SENSOR_H
#define TACHO_INPUT_SENSOR_H

#include "config.h"
#include <sensesp/signalk/signalk_output.h>
#include <sensesp/system/lambda_consumer.h>
#include <sensesp/transforms/lambda_transform.h>
#include <sensesp/sensors/digital_input.h>
#include <sensesp/transforms/time_counter.h>

#include "n2k_senders.h"
#include "sensesp/ui/ui_controls.h"
#include "halmet_digital.h"
#include "halmet_display.h"

using namespace sensesp;

// Tacho input configurations
struct TachoInputConfig {
    int pin;
    const char* input_name;
    const char* sk_path;
    const char* description;
    const char* metadata_units;
    const char* metadata_display_name;
    const char* metadata_description;
    const char* metadata_short_name;
    int initial_sort_order;
};

class TachoInputSensor {
public:
    TachoInputSensor(int pin, const char* input_name, const char* sk_path, 
                     const char* description, const char* metadata_units, 
                     const char* metadata_display_name, const char* metadata_description, 
                     const char* metadata_short_name, int initial_sort_order)
        : pin_(pin), input_name_(input_name), sk_path_(sk_path), description_(description), 
          metadata_units_(metadata_units), metadata_display_name_(metadata_display_name), 
          metadata_description_(metadata_description), metadata_short_name_(metadata_short_name), 
          initial_sort_order_(initial_sort_order) {}

    void connect(N2kEngineParameterRapidSender* n2k_engine_rapid_sender, bool enable_n2k_output, bool display_present, Adafruit_SSD1306* display) {
        String sk_path_enabled = String(sk_path_) + "/Enabled";
        auto* enable_input = new CheckboxConfig(true, "Enable RPM Input", sk_path_enabled);
        enable_input->set_description(String("Enable RPM input ") + input_name_ + ". Requires a reboot to take effect.");
        enable_input->set_sort_order(initial_sort_order_);

        if (enable_input->get_value()) {
            // Connect the tacho sender
            auto tacho_frequency = ConnectTachoSender(pin_, input_name_, "main", initial_sort_order_ + 100);

            // Connect to Signal K output
            tacho_frequency->connect_to(new SKOutput<float>(
                String(sk_path_) + "/Revolutions", String(sk_path_) + "/SK Path",
                new SKMetadata(metadata_units_, metadata_display_name_, metadata_description_, metadata_short_name_)));

            // Engine hours
            auto* engine_hours = new TimeCounter<float>(String(sk_path_) + "/Engine Hours");
            engine_hours->set_description("Engine hours based on the " + String(input_name_) + " tacho input, in seconds.");
            engine_hours->set_sort_order(initial_sort_order_ + 400);
            tacho_frequency->connect_to(engine_hours);

            engine_hours->connect_to(new SKOutput<float>(
                String(sk_path_) + "/Run Time", String(sk_path_) + "/SK Path",
                new SKMetadata("s", "Engine running time", "Main Engine running time")));

            // Propulsion state
            auto* propulsion_state = new LambdaTransform<float, String>([](float freq) {
                if (freq > 0) {
                    return "started";
                } else {
                    return "stopped";
                }
            });

            tacho_frequency->connect_to(propulsion_state);
            propulsion_state->connect_to(new SKOutput<String>(
                String(sk_path_) + "/State", String(sk_path_) + "/SK Path",
                new SKMetadata("", "Engine State", "Main Engine State")));

            if (enable_n2k_output) {
                // Connect outputs to the N2k senders
                tacho_frequency->connect_to(&(n2k_engine_rapid_sender->engine_speed_consumer_));
            }

            if (display_present && display != nullptr) {
                tacho_frequency->connect_to(new LambdaConsumer<float>(
                    [display](float value) { PrintValue(display, 3, "RPM", 60 * value); }));
            }
        }
    }

private:
    int pin_;
    const char* input_name_;
    const char* sk_path_;
    const char* description_;
    const char* metadata_units_;
    const char* metadata_display_name_;
    const char* metadata_description_;
    const char* metadata_short_name_;
    int initial_sort_order_;
};

#endif // TACHO_INPUT_SENSOR_H
