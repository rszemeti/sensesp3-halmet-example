#ifndef DIGITAL_INPUT_SENSOR_H
#define DIGITAL_INPUT_SENSOR_H

#include "config.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_SSD1306.h>

#include "n2k_senders.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/ui/ui_controls.h"

#include "halmet_digital.h"

#include "sensors/sensor_config.h"

extern bool alarm_states[4];

using namespace sensesp;

class DigitalAlarmInput {
public:
    DigitalAlarmInput(int pin, const char* input_name, AlarmType alarm_type, bool inverted)
        : pin_(pin), input_name_(input_name), alarm_type_(alarm_type), inverted_(inverted) {
        initial_sort_order_ = pin_ * 1000 + 6000;
    }

    void connect(N2kEngineParameterDynamicSender* n2k_engine_dynamic_sender, bool enable_n2k_output) {
        auto alarm_input = ConnectAlarmSender(pin_, input_name_,1000);

        // Update the alarm states based on the input value changes.
        if (inverted_) {
            auto alarm_inverted = alarm_input->connect_to(
                new LambdaTransform<bool, bool>([](bool value) { return !value; }));
            alarm_inverted->connect_to(
                new LambdaConsumer<bool>([this](bool value) { alarm_states[pin_] = value; }));
        } else {
            alarm_input->connect_to(
                new LambdaConsumer<bool>([this](bool value) { alarm_states[pin_] = value; }));
        }

        if (enable_n2k_output) {
            // Connect the alarm input to the appropriate N2k consumer.
            switch (alarm_type_) {
                case LOW_OIL:
                    if (inverted_) {
                        auto alarm_inverted = alarm_input->connect_to(
                            new LambdaTransform<bool, bool>([](bool value) { return !value; }));
                        alarm_inverted->connect_to(&(n2k_engine_dynamic_sender->low_oil_pressure_consumer_));
                    } else {
                        alarm_input->connect_to(&(n2k_engine_dynamic_sender->low_oil_pressure_consumer_));
                    }
                    break;
                case OVER_TEMP:
                    if (inverted_) {
                        auto alarm_inverted = alarm_input->connect_to(
                            new LambdaTransform<bool, bool>([](bool value) { return !value; }));
                        alarm_inverted->connect_to(&(n2k_engine_dynamic_sender->over_temperature_consumer_));
                    } else {
                        alarm_input->connect_to(&(n2k_engine_dynamic_sender->over_temperature_consumer_));
                    }
                    break;
            }
        }
    }

private:
    int pin_;
    const char* input_name_;
    AlarmType alarm_type_;
    bool inverted_;
    int initial_sort_order_;
};

#endif // DIGITAL_INPUT_SENSOR_H
