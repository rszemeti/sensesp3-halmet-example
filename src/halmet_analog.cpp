#include "halmet_analog.h"

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"

// ADS1115 input hardware scale factor (input voltage vs voltage at ADS1115)
const float kAnalogInputScale = 29. / 2.048;

// HALMET constant measurement current (A)
const float kMeasurementCurrent = 0.01;


FloatProducer* ConnectAnalogSender(Adafruit_ADS1115* ads1115, int channel,
                                 String name) {
  const uint ads_read_delay = 500;  // ms

  char config_path[80];

  snprintf(config_path, sizeof(config_path), "/Analog %s/Resistance",
           name.c_str());
  auto sender_resistance =
      new RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
      });
  return sender_resistance;
}
