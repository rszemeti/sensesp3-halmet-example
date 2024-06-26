#include "halmet_digital.h"

#include "rate_limiter.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/frequency.h"

using namespace sensesp;

// Default RPM count scale factor, corresponds to 100 pulses per revolution.
// This is rarely, if ever correct.
const float kDefaultFrequencyScale = 1 / 100.;

FloatProducer* ConnectTachoSender(int pin, String path_prefix, String sk_name,
                                  int sort_order_base) {
  char config_path[80];
  char sk_path[80];

  auto tacho_input = new DigitalInputCounter(pin, INPUT, RISING, 500);

  snprintf(config_path, sizeof(config_path), "/%s/Revolution Multiplier",
           path_prefix.c_str());
  auto tacho_frequency = new Frequency(kDefaultFrequencyScale, config_path);
  tacho_frequency->set_description(
      "The ratio of pulses to revolutions <em>per second</em>.");
  tacho_frequency->set_sort_order(sort_order_base + 100);

  snprintf(config_path, sizeof(config_path), "/%s/Revolutions SK Path",
           path_prefix.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.revolutions",
           sk_name.c_str());

  tacho_input->connect_to(tacho_frequency);

#ifdef ENABLE_SIGNALK
  auto tacho_frequency_sk_output = new SKOutputFloat(sk_path, config_path);
  tacho_frequency_sk_output->set_description(
      "Signal K path of the RPM output.");
  tacho_frequency_sk_output->set_sort_order(sort_order_base + 200);

  tacho_frequency->connect_to(tacho_frequency_sk_output);
#endif

  // tacho_input->attach([name, tacho_input]() {
  //   debugD("Input %s counter: %d", name.c_str(), tacho_input->get());
  // });

  return tacho_frequency;
}

BoolProducer* ConnectAlarmSender(int pin, String name, int sort_order_base) {
  char config_path[80];
  char sk_path[80];

  auto* alarm_input = new DigitalInputState(pin, INPUT, 100);

  snprintf(config_path, sizeof(config_path), "/Alarm %s/SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "alarm.%s", name.c_str());

  alarm_input->connect_to(new RateLimiter<bool>(1000));

#ifdef ENABLE_SIGNALK
  auto alarm_sk_output = new SKOutputBool(sk_path, config_path);
  alarm_sk_output->set_description("Signal K path of the alarm output.");
  alarm_sk_output->set_sort_order(sort_order_base + 100);

  alarm_input->connect_to(alarm_sk_output);
#endif

  return alarm_input;
}
