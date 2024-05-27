#ifndef TANK_LEVEL_SENSOR_H
#define TANK_LEVEL_SENSOR_H

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

constexpr char kFillLevelCurveDescription[] =
    "Piecewise linear conversion of the resistance to a "
    "fill level ratio between 0 and 1.</p>"
    "<p>Input values are resistances in ohms, outputs are the corresponding "
    "fill level ratios (between 0 and 1).";

constexpr float kTankA2DefaultSize = 1.0; // Default tank size in m3
constexpr char TANK_METADATA_UNITS[] = "m3";
constexpr char TANK_METADATA_DISPLAY_NAME[] = "Tank Volume";
constexpr char TANK_METADATA_DESCRIPTION[] = "Calculated tank remaining volume";
constexpr char TANK_METADATA_SHORT_NAME[] = "Tank Volume";


extern tNMEA2000* nmea2000; // Ensure this is defined somewhere

using namespace sensesp;

class TankLevelSensor {
public:
    TankLevelSensor(int ads_channel, const char* input_name, const char* sk_path, 
                    const char* description, float tank_size, const char* metadata_units, 
                    const char* metadata_display_name, const char* metadata_description, const char* metadata_short_name)
        : ads_channel_(ads_channel), input_name_(input_name), sk_path_(sk_path), description_(description), 
          tank_size_(tank_size), metadata_units_(metadata_units), metadata_display_name_(metadata_display_name), 
          metadata_description_(metadata_description), metadata_short_name_(metadata_short_name) {
            initial_sort_order_ = ads_channel_ * 1000 +2000;
          }

    void connect(Adafruit_ADS1115* ads, N2kEngineParameterDynamicSender* n2k_engine_dynamic_sender, bool enable_n2k_output, bool display_present, Adafruit_SSD1306* display) {
    
        String sk_path_enabled = String(sk_path_) + "/Enabled";
        auto* enable_input = new CheckboxConfig(false, "Enable Input", sk_path_enabled);
        enable_input->set_description(String("Enable analog input ") + input_name_ + ". Requires a reboot to take effect.");
        enable_input->set_sort_order(initial_sort_order_);

        if (enable_input->get_value()) {
            // Connect the analog sender
            auto analog_resistance = ConnectAnalogSender(ads, ads_channel_, input_name_);

            // Resistance converted to relative value 0..1
            auto level_curve = new CurveInterpolator(nullptr, String(sk_path_) + "/Level Curve");
            level_curve->set_input_title("Sender Resistance (ohms)")
                       ->set_output_title("Fill Level (ratio)")
                       ->set_description(kFillLevelCurveDescription)
                       ->set_sort_order(initial_sort_order_ + 100);
            if (level_curve->get_samples().empty()) {
                // Provide a default curve if there's no prior configuration
                level_curve->clear_samples();
                level_curve->add_sample(CurveInterpolator::Sample(0, 0));
                level_curve->add_sample(CurveInterpolator::Sample(180., 1));
                level_curve->add_sample(CurveInterpolator::Sample(300., 1));
            }
            analog_resistance->connect_to(level_curve);

            // Level converted to remaining volume in m3
            auto volume_transform = new Linear(tank_size_, 0, String(sk_path_) + "/Total Volume");
            volume_transform->set_description("Total volume of tank in litres");
            volume_transform->set_sort_order(initial_sort_order_ + 200);
            level_curve->connect_to(volume_transform);

#ifdef ENABLE_SIGNALK
            auto analog_resistance_sk_output = new SKOutputFloat(
                String("tanks.fuel.") + input_name_ + ".senderResistance", String(sk_path_) + "/Sender Resistance",
                new SKMetadata("ohm", "Input sender resistance", "Input sender resistance"));
            analog_resistance_sk_output->set_sort_order(initial_sort_order_ + 300);
            analog_resistance->connect_to(analog_resistance_sk_output);

            auto level_sk_output = new SKOutputFloat(
                String("tanks.fuel.") + input_name_ + ".currentLevel", String(sk_path_) + "/Current Level",
                new SKMetadata("ratio", "Tank level", "Tank level"));
            level_sk_output->set_sort_order(initial_sort_order_ + 400);
            level_curve->connect_to(level_sk_output);

            auto volume_sk_output = new SKOutputFloat(
                String("tanks.fuel.") + input_name_ + ".currentVolume", String(sk_path_) + "/Current Volume",
                new SKMetadata("m3", "Tank volume", "Calculated tank remaining volume"));
            volume_sk_output->set_sort_order(initial_sort_order_ + 500);
            volume_transform->connect_to(volume_sk_output);
#endif

            if (enable_n2k_output) {
                auto n2k_level_output = new N2kFluidLevelSender(String(sk_path_) + "/NMEA 2000", 1, N2kft_Fuel, 200, nmea2000, enable_n2k_output);
                n2k_level_output->set_sort_order(initial_sort_order_ + 600);
                volume_transform->connect_to(&(n2k_level_output->tank_level_consumer_));
            }

            if (display_present && display != nullptr) {
                volume_transform->connect_to(new LambdaConsumer<float>(
                    [this, display](float value) { PrintValue(display, ads_channel_ + 2, String("Tank ") + input_name_, 100 * value); }));
            }
        }
    }

private:
    int ads_channel_;
    const char* input_name_;
    const char* sk_path_;
    const char* description_;
    float tank_size_;
    const char* metadata_units_;
    const char* metadata_display_name_;
    const char* metadata_description_;
    const char* metadata_short_name_;
    int initial_sort_order_;
};


#endif // TANK_LEVEL_SENSOR_H
