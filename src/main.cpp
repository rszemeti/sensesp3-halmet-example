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

#include "sensors/analogue_sensor_config.h"
#include "sensors/analogue_temperature.h"
#include "sensors/tank_level.h"
#include "sensors/analogue_pressure.h"
#include "sensors/digital_alarm.h"
#include "sensors/onewire_temperature.h"
#include "sensors/tacho.h"

#include "system_config.h"

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

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(kOneWirePin);

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

    SystemConfig system_config;
    if (!system_config.loadFromJson("/config.json")) {
        Serial.println("Failed to load system configuration");
        return;
    }
    // Loop through the sensor configurations and create sensors dynamically
    int sort_order = 1000;
    for (const auto& config : system_config.getAnalogueSensorConfigs()) {
        sort_order += 1000;
        if (config.type == TEMPERATURE_SENSOR) {
            AnalogTemperatureSensor temperature_sensor(
                config.ads_channel, config.input_name, config.sk_path, config.description, config.alarm_path, 
                config.default_limit, config.metadata_units, config.metadata_display_name, config.metadata_description, 
                config.metadata_short_name);
            temperature_sensor.connect(ads1115, n2k_engine_dynamic_sender, enable_n2k_output->get_value(), display_present, display);
        } else if (config.type == TANK_LEVEL_SENSOR) {
            TankLevelSensor tank_sensor(
                config.ads_channel, config.input_name, config.sk_path, config.description, config.tank_size, 
                config.metadata_units, config.metadata_display_name, config.metadata_description, config.metadata_short_name);
            tank_sensor.connect(ads1115, n2k_engine_dynamic_sender, enable_n2k_output->get_value(), display_present, display);
        } else if (config.type == PRESSURE_SENSOR) {
            AnalogPressureSensor pressure_sensor(
                config.ads_channel, config.input_name, config.sk_path, config.description, config.alarm_path, 
                config.default_limit, config.metadata_units, config.metadata_display_name, config.metadata_description, 
                config.metadata_short_name);
            pressure_sensor.connect(ads1115, n2k_engine_dynamic_sender, enable_n2k_output->get_value(), display_present, display);
        }
    }

    // Loop through the tacho input configurations and create sensors dynamically
    /*
    
        for (const auto& config : tacho_input_configs) {
        sort_order += 1000;
        TachoInputSensor tacho_input_sensor(
            config.pin, config.input_name, config.sk_path, config.description, config.metadata_units, 
            config.metadata_display_name, config.metadata_description, config.metadata_short_name, config.initial_sort_order);
        tacho_input_sensor.connect(n2k_engine_rapid_sender, enable_n2k_output->get_value(), display_present, display);
    }

    */


    // Loop through the digital input configurations and create sensors dynamically
    for (const auto& config : system_config.getDigitalInputConfigs()) {
        sort_order += 1000;
        DigitalAlarmInput digital_input(config.pin, config.input_name, config.alarm_type, config.inverted);
        digital_input.connect(n2k_engine_dynamic_sender, enable_n2k_output->get_value());
    }

    // Loop through the OneWire temperature sensor configurations and create sensors dynamically
    for (const auto& config : system_config.getOneWireTemperatureConfigs() ) {
        sort_order += 1000;
        OneWireTemperatureSensor onewire_temperature_sensor(
            config.input_name, config.sk_path, config.description, config.alarm_path, config.default_limit, 
            config.metadata_units, config.metadata_display_name, config.metadata_description, config.metadata_short_name, 
            config.initial_sort_order);
        onewire_temperature_sensor.connect(dts, n2k_engine_dynamic_sender, enable_n2k_output->get_value());
    }

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

    }

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
