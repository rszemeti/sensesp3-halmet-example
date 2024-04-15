// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

// Comment out this line to disable NMEA 2000 output.
#define ENABLE_NMEA2000_OUTPUT

// Comment out this line to disable Signal K support. At the moment, disabling
// Signal K support also disables all WiFi functionality.
#define ENABLE_SIGNALK

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifdef ENABLE_NMEA2000_OUTPUT
#include <NMEA2000_esp32.h>
#endif

#include "n2k_senders.h"
#include "sensesp/net/discovery.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/system_status_led.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/time_counter.h"
#include "sensesp/ui/ui_controls.h"
#include "sensesp_onewire/onewire_temperature.h"

#ifdef ENABLE_SIGNALK
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder
#else
#include "sensesp_minimal_app_builder.h"
#endif

#include "halmet_analog.h"
#include "halmet_any.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "n2k_senders.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"

using namespace sensesp;

#ifndef ENABLE_SIGNALK
#define BUILDER_CLASS SensESPMinimalAppBuilder
SensESPMinimalApp* sensesp_app;
Networking* networking;
MDNSDiscovery* mdns_discovery;
HTTPServer* http_server;
SystemStatusLed* system_status_led;
#endif

/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.

// Default fuel tank size, in m3
const float kTankDefaultSize = 120. / 1000;

constexpr int kOneWirePin = GPIO_NUM_4;

#ifdef ENABLE_NMEA2000_OUTPUT
tNMEA2000* nmea2000;
#endif

TwoWire* i2c;
Adafruit_SSD1306* display;

reactesp::ReactESP app;

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

/////////////////////////////////////////////////////////////////////
// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.
const int kTestOutputFrequency = 380;
#endif

constexpr char kFillLevelCurveDescription[] =
    "Piecewise linear conversion of the resistance to a "
    "fill level ratio between 0 and 1.</p>"
    "<p>Input values are resistances in ohms, outputs are the corresponding "
    "fill level ratios (between 0 and 1).";

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(GAIN_ONE);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  // Set the LEDC peripheral to a 13-bit resolution
  ledcSetup(0, kTestOutputFrequency, 13);
  // Attach the channel to the GPIO pin to be controlled
  ledcAttachPin(kTestOutputPin, 0);
  // Set the duty cycle to 50%
  // Duty cycle value is calculated based on the resolution
  // For 13-bit resolution, max value is 8191, so 50% is 4096
  ledcWrite(0, 4096);
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20231229",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "HALMET",    // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    71  // Default N2k node address
  );
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() { nmea2000->ParseMessages(); });
#endif  // ENABLE_NMEA2000_OUTPUT

  /////////////////////////////////////////////////////////////////////
  // Initialize the application framework

  // Construct the global SensESPApp() object
  BUILDER_CLASS builder;
  sensesp_app = (&builder)
                    // EDIT: Set a custom hostname for the app.
                    ->set_hostname("sensesp3-halmet-example")
                    ->enable_ota("gishaaquav1O")
                    // EDIT: Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

#ifndef ENABLE_SIGNALK
  // Initialize components that would normally be present in SensESPApp
  networking = new Networking("/System/WiFi Settings", "", "",
                              SensESPBaseApp::get_hostname(), "thisisfine");
  mdns_discovery = new MDNSDiscovery();
  http_server = new HTTPServer();
  system_status_led = new SystemStatusLed(LED_BUILTIN);
#endif

  // Initialize the OLED display
  bool display_present = InitializeSSD1306(&app, sensesp_app, &display, i2c);

  /////////////////////////////////////////////////////////////////////
  // NMEA 2000 sender objects

  auto* enable_n2k_output = new CheckboxConfig(
      false, "NMEA 2000 Output Enabled", "/NMEA 2000/NMEA 2000 Enabled");
  enable_n2k_output->set_description(
      "Enable NMEA 2000 output. If disabled, no NMEA 2000 "
      "messages will be sent, regardless of other settings.");
  enable_n2k_output->set_sort_order(500);

  N2kEngineParameterRapidSender* n2k_engine_rapid_sender;
  N2kEngineParameterDynamicSender* n2k_engine_dynamic_sender;

  if (enable_n2k_output->get_value()) {
    // Create the NMEA 2000 sender objects

    n2k_engine_rapid_sender = new N2kEngineParameterRapidSender(
        "/NMEA 2000/Engine Rapid Update", 0, nmea2000,
        enable_n2k_output->get_value());
    n2k_engine_rapid_sender->set_description(
        "PGN 127488 (Engine Rapid Update) parameters.");
    n2k_engine_rapid_sender->set_sort_order(510);

    n2k_engine_dynamic_sender = new N2kEngineParameterDynamicSender(
        "/NMEA 2000/Engine Dynamic", 0, nmea2000,
        enable_n2k_output->get_value());
    n2k_engine_dynamic_sender->set_sort_order(520);
  }

  ///////////////////////////////////////////////////////////////////
  // Analog inputs

  // Analog input A1

  auto* enable_a1_input =
      new CheckboxConfig(true, "Enable A1 Input", "/Tank A1/Enabled");
  enable_a1_input->set_description(
      "Enable analog tank level input A1. Requires a reboot to take effect.");
  enable_a1_input->set_sort_order(1000);

  if (enable_a1_input->get_value()) {
    // Connect the tank senders.
    auto analog_a1_resistance = ConnectAnalogSender(ads1115, 0, "A1");
    // Resistance converted to relative value 0..1
    auto tank_a1_level =
        (new CurveInterpolator(nullptr, "/Tank A1/Level Curve"));
    tank_a1_level->set_input_title("Sender Resistance (ohms)")
        ->set_output_title("Fill Level (ratio)")
        ->set_description(kFillLevelCurveDescription)
        ->set_sort_order(1100);
    if (tank_a1_level->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      tank_a1_level->clear_samples();
      tank_a1_level->add_sample(CurveInterpolator::Sample(0, 0));
      tank_a1_level->add_sample(CurveInterpolator::Sample(180., 1));
    }
    analog_a1_resistance->connect_to(tank_a1_level);

    // Level converted to remaining volume in m3
    auto tank_a1_volume =
        new Linear(kTankDefaultSize, 0, "/Tank A1/Total Volume");
    tank_a1_volume->set_description("Total volume of tank A1 in m3");
    tank_a1_volume->set_sort_order(1200);
    tank_a1_level->connect_to(tank_a1_volume);
#ifdef ENABLE_SIGNALK
    auto analog_a1_resistance_sk_output = new SKOutputFloat(
        "tanks.fuel.A1.senderResistance", "/Tank A1/Sender Resistance",
        new SKMetadata("ohm", "Input A1 sender resistance",
                       "Input A1 sender resistance"));
    analog_a1_resistance_sk_output->set_sort_order(1300);
    analog_a1_resistance->connect_to(analog_a1_resistance_sk_output);

    auto tank_a1_level_sk_output = new SKOutputFloat(
        "tanks.fuel.A1.currentLevel", "/Tank A1/Current Level",
        new SKMetadata("ratio", "Tank A1 level", "Tank A1 level"));
    tank_a1_level_sk_output->set_sort_order(1400);
    tank_a1_level->connect_to(tank_a1_level_sk_output);

    auto tank_a1_volume_sk_output = new SKOutputFloat(
        "tanks.fuel.A1.currentVolume", "/Tank A1/Current Volume",
        new SKMetadata("m3", "Tank A1 volume",
                       "Calculated tank A1 remaining volume"));
    tank_a1_volume_sk_output->set_sort_order(1500);
    tank_a1_volume->connect_to(tank_a1_volume_sk_output);
#endif

    if (enable_n2k_output->get_value()) {
      auto tank_a1_n2k_level_output =
          new N2kFluidLevelSender("/Tank A1/NMEA 2000", 0, N2kft_Fuel, 200,
                                  nmea2000, enable_n2k_output->get_value());
      tank_a1_n2k_level_output->set_sort_order(1600);
      tank_a1_volume->connect_to(
          &(tank_a1_n2k_level_output->tank_level_consumer_));
    }
    if (display_present) {
      // EDIT: Duplicate the lines below to make the display show all your
      // tanks.
      tank_a1_volume->connect_to(new LambdaConsumer<float>(
          [](float value) { PrintValue(display, 2, "Tank A1", 100 * value); }));
    }

  }  // if (enable_a1_input->get_value())

  // Analog input A2

  auto* enable_a2_input =
      new CheckboxConfig(false, "Enable A2 Input", "/Tank A2/Enabled");
  enable_a2_input->set_description(
      "Enable analog tank level input A2. Requires a reboot to take effect.");
  enable_a2_input->set_sort_order(2000);

  if (enable_a2_input->get_value()) {
    // Connect the tank senders.
    auto analog_a2_resistance = ConnectAnalogSender(ads1115, 1, "A2");
    // Resistance converted to relative value 0..1
    auto tank_a2_level =
        (new CurveInterpolator(nullptr, "/Tank A2/Level Curve"))
            ->set_input_title("Sender Resistance (ohms)")
            ->set_output_title("Fill Level (ratio)");
    tank_a2_level->set_description(kFillLevelCurveDescription);
    tank_a2_level->set_sort_order(2100);
    if (tank_a2_level->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      tank_a2_level->clear_samples();
      tank_a2_level->add_sample(CurveInterpolator::Sample(0, 0));
      tank_a2_level->add_sample(CurveInterpolator::Sample(180., 1));
      tank_a2_level->add_sample(CurveInterpolator::Sample(300., 1));
    }
    analog_a2_resistance->connect_to(tank_a2_level);
    // Level converted to remaining volume in m3
    auto tank_a2_volume =
        new Linear(kTankDefaultSize, 0, "/Tank A2/Total Volume");
    tank_a2_volume->set_description("Total volume of tank A2 in m3");
    tank_a2_volume->set_sort_order(2200);
    tank_a2_level->connect_to(tank_a2_volume);
#ifdef ENABLE_SIGNALK
    auto analog_a2_resistance_sk_output = new SKOutputFloat(
        "tanks.fuel.A2.senderResistance", "/Tank A2/Sender Resistance",
        new SKMetadata("ohm", "Input A2 sender resistance",
                       "Input A2 sender resistance"));
    analog_a2_resistance_sk_output->set_sort_order(2300);
    analog_a2_resistance->connect_to(analog_a2_resistance_sk_output);
    auto tank_a2_level_sk_output = new SKOutputFloat(
        "tanks.fuel.A2.currentLevel", "/Tank A2/Current Level",
        new SKMetadata("ratio", "Tank A2 level", "Tank A2 level"));
    tank_a2_level_sk_output->set_sort_order(2400);
    tank_a2_level->connect_to(tank_a2_level_sk_output);
    auto tank_a2_volume_sk_output = new SKOutputFloat(
        "tanks.fuel.A2.currentVolume", "/Tank A2/Current Volume",
        new SKMetadata("m3", "Tank A2 volume",
                       "Calculated tank A2 remaining volume"));
    tank_a2_volume_sk_output->set_sort_order(2500);
    tank_a2_volume->connect_to(tank_a2_volume_sk_output);
#endif

    if (enable_n2k_output->get_value()) {
      auto tank_a2_n2k_level_output =
          new N2kFluidLevelSender("/Tank A2/NMEA 2000", 1, N2kft_Water, 200,
                                  nmea2000, enable_n2k_output->get_value());
      tank_a2_n2k_level_output->set_sort_order(2600);
      tank_a2_volume->connect_to(
          &(tank_a2_n2k_level_output->tank_level_consumer_));
    }
  }  // if (enable_a2_input->get_value())

  // Analog input A3

  auto* enable_a3_input =
      new CheckboxConfig(false, "Enable A3 Input", "/Tank A3/Enabled");
  enable_a3_input->set_description(
      "Enable analog tank level input A3. Requires a reboot to take effect.");
  enable_a3_input->set_sort_order(3000);

  if (enable_a3_input->get_value()) {
    // Connect the tank senders.
    auto analog_a3_resistance = ConnectAnalogSender(ads1115, 2, "A3");
    // Resistance converted to relative value 0..1
    auto tank_a3_level =
        (new CurveInterpolator(nullptr, "/Tank A3/Level Curve"))
            ->set_input_title("Sender Resistance (ohms)")
            ->set_output_title("Fill Level (ratio)");
    tank_a3_level->set_description(kFillLevelCurveDescription);
    tank_a3_level->set_sort_order(3100);
    if (tank_a3_level->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      tank_a3_level->clear_samples();
      tank_a3_level->add_sample(CurveInterpolator::Sample(0, 0));
      tank_a3_level->add_sample(CurveInterpolator::Sample(180., 1));
      tank_a3_level->add_sample(CurveInterpolator::Sample(300., 1));
    }
    analog_a3_resistance->connect_to(tank_a3_level);
    // Level converted to remaining volume in m3
    auto tank_a3_volume =
        new Linear(kTankDefaultSize, 0, "/Tank A3/Total Volume");
    tank_a3_volume->set_description("Total volume of tank A3 in m3");
    tank_a3_volume->set_sort_order(3200);
    tank_a3_level->connect_to(tank_a3_volume);
#ifdef ENABLE_SIGNALK
    auto analog_a3_resistance_sk_output = new SKOutputFloat(
        "tanks.fuel.A3.senderResistance", "/Tank A3/Sender Resistance",
        new SKMetadata("ohm", "Input A3 sender resistance",
                       "Input A3 sender resistance"));
    analog_a3_resistance_sk_output->set_sort_order(3300);
    analog_a3_resistance->connect_to(analog_a3_resistance_sk_output);
    auto tank_a3_level_sk_output = new SKOutputFloat(
        "tanks.fuel.A3.currentLevel", "/Tank A3/Current Level",
        new SKMetadata("ratio", "Tank A3 level", "Tank A3 level"));
    tank_a3_level_sk_output->set_sort_order(3400);
    tank_a3_level->connect_to(tank_a3_level_sk_output);
    auto tank_a3_volume_sk_output = new SKOutputFloat(
        "tanks.fuel.A3.currentVolume", "/Tank A3/Current Volume",
        new SKMetadata("m3", "Tank A3 volume",
                       "Calculated tank A3 remaining volume"));
    tank_a3_volume_sk_output->set_sort_order(3500);
    tank_a3_volume->connect_to(tank_a3_volume_sk_output);
#endif
    if (enable_n2k_output->get_value()) {
      auto tank_a3_n2k_level_output =
          new N2kFluidLevelSender("/Tank A3/NMEA 2000", 2, N2kft_GrayWater, 200,
                                  nmea2000, enable_n2k_output->get_value());
      tank_a3_n2k_level_output->set_sort_order(3600);
      tank_a3_volume->connect_to(
          &(tank_a3_n2k_level_output->tank_level_consumer_));
    }
  }  // if (enable_a3_input->get_value())

  // Analog input A4

  auto* enable_a4_input =
      new CheckboxConfig(false, "Enable A4 Input", "/Pressure A4/Enabled");
  enable_a4_input->set_description(
      "Enable analog pressure input A4. Requires a reboot to take effect.");
  enable_a4_input->set_sort_order(4000);

  if (enable_a4_input->get_value()) {
    // Connect the pressure sender.
    auto analog_a4_resistance = ConnectAnalogSender(ads1115, 3, "A4");
    // Resistance converted to pressure in bar
    auto sender_a4_pressure =
        (new CurveInterpolator(nullptr, "/Pressure A4/Pressure"))
            ->set_input_title("Sender Resistance (ohms)")
            ->set_output_title("Pressure (Pa)");
    sender_a4_pressure->set_description(
        "Piecewise linear conversion of the resistance of the A4 sender to a "
        "pressure in Pascal. Input is resistance, output is pressure in "
        "Pascal.");
    sender_a4_pressure->set_sort_order(4100);
    if (sender_a4_pressure->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      sender_a4_pressure->clear_samples();
      sender_a4_pressure->add_sample(CurveInterpolator::Sample(0, 0));
      sender_a4_pressure->add_sample(CurveInterpolator::Sample(180., 300000));
      sender_a4_pressure->add_sample(CurveInterpolator::Sample(300., 300000));
    }
    analog_a4_resistance->connect_to(sender_a4_pressure);

    const ParamInfo* low_pressure_limit =
        new ParamInfo[1]{{"low_pressure_limit", "Low Pressure Limit"}};

    auto alarm_function = [](float pressure, float limit) -> bool {
      return pressure < limit;
    };

    auto sender_a4_low_pressure_alarm = new LambdaTransform<float, bool, float>(
        alarm_function,
        100000,              // Default value for parameter
        low_pressure_limit,  // Parameter UI description
        "/Pressure A4/Low Pressure Alarm");
    sender_a4_low_pressure_alarm->set_description(
        "Alarm if the pressure falls below the set limit. Value in Pascal.");
    sender_a4_low_pressure_alarm->set_sort_order(4150);
    sender_a4_pressure->connect_to(sender_a4_low_pressure_alarm);

#ifdef ENABLE_SIGNALK
    auto analog_a4_resistance_sk_output =
        new SKOutputFloat("propulsion.main.oilPressureSenderResistance",
                          "/Pressure A4/Sender Resistance",
                          new SKMetadata("ohm", "Input A4 sender resistance",
                                         "Input A4 sender resistance"));
    analog_a4_resistance_sk_output->set_sort_order(4200);
    analog_a4_resistance->connect_to(analog_a4_resistance_sk_output);
    auto sender_a4_pressure_sk_output = new SKOutputFloat(
        "propulsion.main.oilPressure", "/Pressure A4/Current Pressure",
        new SKMetadata("Pa", "Oil Pressure", "Main Engine Oil Pressure"));
    sender_a4_pressure_sk_output->set_sort_order(4300);
    sender_a4_pressure->connect_to(sender_a4_pressure_sk_output);
#endif
    if (enable_n2k_output->get_value()) {
      // Connect the pressure output to N2k dynamic sender
      sender_a4_pressure->connect_to(
          &(n2k_engine_dynamic_sender->oil_pressure_consumer_));

      // Connect the low pressure alarm to N2k dynamic sender
      sender_a4_low_pressure_alarm->connect_to(
          &(n2k_engine_dynamic_sender->low_oil_pressure_consumer_));
    }
  }  // if (enable_a4_input->get_value())

  ///////////////////////////////////////////////////////////////////
  // Digital tacho inputs

  auto* enable_d1_rpm_output =
      new CheckboxConfig(true, "Enable RPM Output", "/Tacho D1/Enabled");
  enable_d1_rpm_output->set_description(
      "Enable RPM input D1. Requires a reboot to take effect.");
  enable_d1_rpm_output->set_sort_order(5000);

  if (enable_d1_rpm_output->get_value()) {
    // Connect the tacho senders. Engine name is "main".
    auto tacho_d1_frequency =
        ConnectTachoSender(kDigitalInputPin1, "Tacho D1", "main", 5100);

    tacho_d1_frequency->connect_to(
        new SKOutput<float>("propulsion.main.revolutions", "",
                            new SKMetadata("Hz", "Main Engine Revolutions")));

    auto* engine_hours = new TimeCounter<float>("/Tacho D1/Engine Hours");
    engine_hours->set_description(
        "Engine hours based on the D1 tacho input, in seconds.");
    engine_hours->set_sort_order(5400);
    tacho_d1_frequency->connect_to(engine_hours);

    // create and connect the engine hours output object
    engine_hours->connect_to(
        new SKOutput<float>("propulsion.main.runTime", "",
                            new SKMetadata("s", "Main Engine running time")));

    // create a propulsion state lambda transform
    auto* propulsion_state = new LambdaTransform<float, String>([](bool freq) {
      if (freq > 0) {
        return "started";
      } else {
        return "stopped";
      }
    });

    // connect the tacho frequency to the propulsion state lambda transform
    tacho_d1_frequency->connect_to(propulsion_state);
    // create and connect the propulsion state output object
    propulsion_state->connect_to(new SKOutput<String>(
        "propulsion.main.state", "", new SKMetadata("", "Main Engine State")));

    if (enable_n2k_output->get_value()) {
      // Connect outputs to the N2k senders.
      tacho_d1_frequency->connect_to(
          &(n2k_engine_rapid_sender->engine_speed_consumer_));
    }

    if (display_present) {
      tacho_d1_frequency->connect_to(new LambdaConsumer<float>(
          [](float value) { PrintValue(display, 3, "RPM D1", 60 * value); }));
    }
  }

  ///////////////////////////////////////////////////////////////////
  // Digital alarm inputs

#if false

  // EDIT: More alarm inputs can be defined by duplicating the lines below.
  // Make sure to not define a pin for both a tacho and an alarm.
  auto alarm_d2_input = ConnectAlarmSender(kDigitalInputPin2, "D2");
  auto alarm_d3_input = ConnectAlarmSender(kDigitalInputPin3, "D3");
  // auto alarm_d4_input = ConnectAlarmSender(kDigitalInputPin4, "D4");

  // Update the alarm states based on the input value changes.
  // EDIT: If you added more alarm inputs, uncomment the respective lines below.
  alarm_d2_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  // In this example, alarm_d3_input is active low, so invert the value.
  auto alarm_d3_inverted = alarm_d3_input->connect_to(
      new LambdaTransform<bool, bool>([](bool value) { return !value; }));
  alarm_d3_inverted->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  // alarm_d4_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));

  if (enable_n2k_output->get_value()) {
    // EDIT: This example connects the D2 alarm input to the low oil pressure
    // warning. Modify according to your needs.
    N2kEngineParameterDynamicSender* engine_dynamic_sender =
        new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0,
                                            nmea2000);
    alarm_d2_input->connect_to(
        &(engine_dynamic_sender->low_oil_pressure_consumer_));
    // This is just an example -- normally temperature alarms would not be
    // active-low (inverted).
    alarm_d3_inverted->connect_to(
        &(engine_dynamic_sender->over_temperature_consumer_));
  }

#endif

  ///////////////////////////////////////////////////////////////////
  // 1-Wire Temperature Sensors

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(kOneWirePin);

  auto main_engine_oil_temperature_metadata =
      new SKMetadata("K",                       // units
                     "Engine Oil Temperature",  // display name
                     "Engine Oil Temperature",  // description
                     "Oil Temperature",         // short name
                     10.                        // timeout, in seconds
      );
  auto main_engine_coolant_temperature_metadata =
      new SKMetadata("K",                           // units
                     "Engine Coolant Temperature",  // display name
                     "Engine Coolant Temperature",  // description
                     "Coolant Temperature",         // short name
                     10.                            // timeout, in seconds
      );
  auto main_engine_temperature_metadata =
      new SKMetadata("K",                   // units
                     "Engine Temperature",  // display name
                     "Engine Temperature",  // description
                     "Temperature",         // short name
                     10.                    // timeout, in seconds
      );
  auto main_engine_exhaust_temperature_metadata =
      new SKMetadata("K",                        // units
                     "Wet Exhaust Temperature",  // display name
                     "Wet Exhaust Temperature",  // description
                     "Exhaust Temperature",      // short name
                     10.                         // timeout, in seconds
      );

  // 1-Wire temperature sensor 1

  OneWireTemperature* main_engine_oil_temperature =
      new OneWireTemperature(dts, 1000, "/Temperature 1/OneWire");
  main_engine_oil_temperature->set_description(
      "Engine oil temperature sensor on the 1-Wire bus.");
  main_engine_oil_temperature->set_sort_order(6000);

  // connect the sensors to Signal K output paths

  auto oil_temp_sk_output = new SKOutput<float>(
      "propulsion.main.oilTemperature", "/Temperature 1/SK Path",
      main_engine_oil_temperature_metadata);
  oil_temp_sk_output->set_sort_order(6100);
  main_engine_oil_temperature->connect_to(oil_temp_sk_output);

  main_engine_oil_temperature->connect_to(
      &(n2k_engine_dynamic_sender->oil_temperature_consumer_));

  Any* temp_alarm = new Any(2);
  temp_alarm->connect_to(
      &(n2k_engine_dynamic_sender->over_temperature_consumer_));

  const ParamInfo* oil_temperature_limit =
      new ParamInfo[1]{{"oil_temperature_limit", "Oil Temperature Limit"}};

  const auto oil_temp_alarm_function = [](float temperature,
                                          float limit) -> bool {
    return temperature > limit;
  };

  auto sender_oil_temp_alarm = new LambdaTransform<float, bool, float>(
      oil_temp_alarm_function,
      383,                    // Default value for parameter
      oil_temperature_limit,  // Parameter UI description
      "/Temperature 1/Oil Temperature Alarm");
  sender_oil_temp_alarm->set_description(
      "Alarm if the oil temperature exceeds the set limit. Value in Kelvin.");
  sender_oil_temp_alarm->set_sort_order(6200);
  main_engine_oil_temperature->connect_to(sender_oil_temp_alarm);

  if (enable_n2k_output->get_value()) {
    // Connect the oil temperature output to N2k dynamic sender
    main_engine_oil_temperature->connect_to(
        &(n2k_engine_dynamic_sender->oil_temperature_consumer_));

    // Connect the oil temperature alarm to N2k dynamic sender
    sender_oil_temp_alarm->connect_to(temp_alarm, 0);
  }

  // 1-Wire temperature sensor 2

  OneWireTemperature* main_engine_coolant_temperature =
      new OneWireTemperature(dts, 1000, "/Temperature 2/OneWire");
  main_engine_coolant_temperature->set_description(
      "Engine coolant temperature sensor on the 1-Wire bus.");
  main_engine_coolant_temperature->set_sort_order(7000);

  main_engine_coolant_temperature->connect_to(
      &(n2k_engine_dynamic_sender->temperature_consumer_));

  auto main_engine_coolant_temperature_sk_output =
      new SKOutput<float>("propulsion.main.coolantTemperature",
                          "/Temperature 2/Coolant Temperature SK Path",
                          main_engine_coolant_temperature_metadata);
  main_engine_coolant_temperature_sk_output->set_sort_order(7100);
  auto main_engine_temperature_sk_output = new SKOutput<float>(
      "propulsion.main.temperature", "/Temperature 2/Temperature SK Path",
      main_engine_temperature_metadata);
  main_engine_temperature_sk_output->set_sort_order(7200);

  main_engine_coolant_temperature->connect_to(
      main_engine_coolant_temperature_sk_output);
  // transmit coolant temperature as overall engine temperature as well
  main_engine_coolant_temperature->connect_to(
      main_engine_temperature_sk_output);

  const ParamInfo* coolant_temperature_limit = new ParamInfo[1]{
      {"coolant_temperature_limit", "Coolant Temperature Limit"}};

  const auto coolant_temp_alarm_function = [](float temperature,
                                              float limit) -> bool {
    return temperature > limit;
  };

  auto sender_coolant_temp_alarm = new LambdaTransform<float, bool, float>(
      coolant_temp_alarm_function,
      373,                        // Default value for parameter
      coolant_temperature_limit,  // Parameter UI description
      "/Temperature 2/Coolant Temperature Alarm");
  sender_coolant_temp_alarm->set_description(
      "Alarm if the coolant temperature exceeds the set limit. Value in "
      "Kelvin.");
  sender_coolant_temp_alarm->set_sort_order(6200);
  main_engine_coolant_temperature->connect_to(sender_coolant_temp_alarm);

  if (enable_n2k_output->get_value()) {
    // Connect the coolant temperature output to N2k dynamic sender
    main_engine_coolant_temperature->connect_to(
        &(n2k_engine_dynamic_sender->temperature_consumer_));

    // Connect the coolant temperature alarm to N2k dynamic sender
    sender_coolant_temp_alarm->connect_to(temp_alarm, 0);
  }

  // 1-Wire temperature sensor 3

  OneWireTemperature* main_engine_exhaust_temperature =
      new OneWireTemperature(dts, 1000, "/Temperature 3/OneWire");
  main_engine_exhaust_temperature->set_sort_order(8000);
  main_engine_exhaust_temperature->set_description(
      "Engine wet exhaust temperature sensor on the 1-Wire bus.");
  main_engine_exhaust_temperature->set_sort_order(8100);

  auto main_engine_exhaust_temperature_sk_path = new SKOutput<float>(
      "propulsion.main.wetExhaustTemperature", "/Temperature 3/SK Path",
      main_engine_exhaust_temperature_metadata);
  main_engine_exhaust_temperature_sk_path->set_sort_order(8200);
  // propulsion.*.wetExhaustTemperature is a non-standard path
  main_engine_exhaust_temperature->connect_to(
      main_engine_exhaust_temperature_sk_path);

  const ParamInfo* exhaust_temperature_limit = new ParamInfo[1]{
      {"exhaust_temperature_limit", "Exhaust Temperature Limit"}};

  const auto exhaust_temp_alarm_function = [](float temperature,
                                              float limit) -> bool {
    return temperature > limit;
  };

  auto sender_exhaust_temp_alarm = new LambdaTransform<float, bool, float>(
      exhaust_temp_alarm_function,
      333,                        // Default value for parameter
      exhaust_temperature_limit,  // Parameter UI description
      "/Temperature 3/Coolant Temperature Alarm");
  sender_exhaust_temp_alarm->set_description(
      "Alarm if the coolant temperature exceeds the set limit. Value in "
      "Kelvin.");
  sender_exhaust_temp_alarm->set_sort_order(8300);
  main_engine_exhaust_temperature->connect_to(sender_exhaust_temp_alarm);

  N2kTemperatureExtSender* n2k_exhaust_temp_sender =
      new N2kTemperatureExtSender("/Temperature 3/NMEA 2000", 0,
                                  N2kts_ExhaustGasTemperature, nmea2000,
                                  enable_n2k_output->get_value());
  n2k_exhaust_temp_sender->set_sort_order(8400);

  if (enable_n2k_output->get_value()) {
    // Connect the coolant temperature output to N2k dynamic sender
    main_engine_exhaust_temperature->connect_to(
        &(n2k_engine_dynamic_sender->temperature_consumer_));

    // Connect the coolant temperature alarm to N2k dynamic sender
    sender_exhaust_temp_alarm->connect_to(temp_alarm, 0);
  }

  // FIXME: Transmit the alarms over SK as well.

  ///////////////////////////////////////////////////////////////////
  // Display setup

  // Connect the outputs to the display
  if (display_present) {
#ifdef ENABLE_SIGNALK
    app.onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });
#endif

    // Create a poor man's "christmas tree" display for the alarms
    app.onRepeat(1000, []() {
      char state_string[5] = {};
      for (int i = 0; i < 4; i++) {
        state_string[i] = alarm_states[i] ? '*' : '_';
      }
      PrintValue(display, 4, "Alarm", state_string);
    });
  }

  ///////////////////////////////////////////////////////////////////
  // Start the application

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
