#ifndef DIGITAL_ALARM_INPUT_H
#define DIGITAL_ALARM_INPUT_H


#include "n2k_senders.h"


template<int PIN, const char* INPUT_NAME, int STATE_INDEX, int INITIAL_SORT_ORDER>
void setupDigitalAlarmSensor( N2kEngineParameterDynamicSender consumer, bool active_low = false) {
  // Connect the alarm sender
  auto alarm_input = ConnectAlarmSender(PIN, INPUT_NAME);

  // Update the alarm states based on the input value changes
  if (active_low) {
    auto alarm_inverted = alarm_input->connect_to(
        new LambdaTransform<bool, bool>([](bool value) { return !value; }));
    alarm_inverted->connect_to(
        new LambdaConsumer<bool>([](bool value) { alarm_states[STATE_INDEX] = value; }));
  } else {
    alarm_input->connect_to(
        new LambdaConsumer<bool>([](bool value) { alarm_states[STATE_INDEX] = value; }));
  }

  if (enable_n2k_output->get_value()) {
    // Connect the alarm input to N2k dynamic sender
    N2kEngineParameterDynamicSender* engine_dynamic_sender =
        new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0, nmea2000);

    if (active_low) {
      auto alarm_inverted = alarm_input->connect_to(
          new LambdaTransform<bool, bool>([](bool value) { return !value; }));
      alarm_inverted->connect_to(&consumer);
    } else {
      alarm_input->connect_to(&consumer);
    }
  }
}

#endif // DIGITAL_ALARM_INPUT_H
