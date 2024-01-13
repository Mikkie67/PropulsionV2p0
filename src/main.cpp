// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

// #define SERIAL_DEBUG_DISABLED
// #define OTA_ENABLED

#include "sensesp/controllers/smart_switch_controller.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_listener.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_put_request_listener.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
// for physical button
#include "sensesp/transforms/click_type.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/press_repeater.h"
#include "sensesp/transforms/repeat_report.h"

using namespace sensesp;
// 1-Wire data pin on SH-ESP32
// ESP32 pins are specified as just the X in GPIOX
#define ONEWIRE_PIN 15
#define COOLANT_FAN_BUTTON 13
#define COOLANT_PUMP_BUTTON 12
#define AMBIENT_FAN_BUTTON 14
#define COOLANT_FAN_RELAY 27
#define COOLANT_PUMP_RELAY 26
#define AMBIENT_FAN_RELAY 25

float KelvinToCelsius(float temp) { return temp - 273.15; }
double portMotor_temperature = -128;
double portController_temperature = -128;
double starboardMotor_temperature = -128;
double starboardController_temperature = -128;
double engineroom_temperature = -128;
double coolant_temperature = -128;

reactesp::ReactESP app;
// ----------------------------------------------------------
// Interrupt service routine
// ----------------------------------------------------------
// Timestamp of the last pulse, in microseconds
volatile unsigned long lastTime = 0;
// Timestamp of the current pulse, in microseconds
volatile unsigned long currentTime = 0;
// RPM value
volatile float shaftHz = 0;
volatile unsigned long timeDifference = 0;
volatile bool updateRpm = false;
ICACHE_RAM_ATTR void isr() {
  // Update currentTime with the current time in microseconds
  currentTime = micros();
  updateRpm = true;
}

// ----------------------------------------------------------
// The setup function performs one-time application initialization.
// ----------------------------------------------------------
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Kerosheba-Propulsion1")
// Optionally, hard-code the WiFi and Signal K server
// settings. This is normally not needed.
//->set_wifi("My WiFi SSID", "my_wifi_password")
//->set_sk_server("192.168.10.3", 80)
#ifdef OTA_ENABLED
                    ->enable_ota("Regurk67!")
#endif
                    ->get_app();

  // Set GPIO pin 22 to output and toggle it every 20 ms
  const uint8_t kDigitalOutputPin = 22;
  const unsigned int kDigitalOutputInterval = 20;
  pinMode(kDigitalOutputPin, OUTPUT);
  app.onRepeat(kDigitalOutputInterval, [kDigitalOutputPin]() {
    digitalWrite(kDigitalOutputPin, !digitalRead(kDigitalOutputPin));
  });

  // -------------------------------------------------------------
  // KEROSHEBA PROPULSION ITEMS
  // -------------------------------------------------------------
  // -------------------------------------------------------------
  // COOLANT FAN, PUMP AND AMBIENT FAN (ENGINE ROOM FAN)
  // -------------------------------------------------------------
  const char* sk_path_coolant_fan = "propulsion.switches.coolant.fan.state";
  const char* sk_path_coolant_pump = "propulsion.switches.coolant.pump.state";
  const char* sk_path_ambient_fan = "propulsion.switches.ambient.fan.state";
  // "Configuration paths" are combined with "/config" to formulate a URL
  // used by the RESTful API for retrieving or setting configuration data.
  // It is ALSO used to specify a path to the SPIFFS file system
  // where configuration data is saved on the MCU board.  It should
  // ALWAYS start with a forward slash if specified.  If left blank,
  // that indicates a sensor or transform does not have any
  // configuration to save.
  const char* config_path_button_c = "/button/clicktime";
  const char* config_path_status_light = "/button/statusLight";
  const char* config_path_sk_output_coolant_fan = "/signalk/path/coolant/fan";
  const char* config_path_sk_output_coolant_pmp = "/signalk/path/coolant/pump";
  const char* config_path_sk_output_ambient_fan = "/signalk/path/ambient/fan";
  const char* config_path_repeat = "/signalk/repeat";

  // Create a digital output that is assumed to be connected to the
  // control channel of a relay or a MOSFET that will control the
  // coolant fan.
  pinMode(COOLANT_FAN_RELAY, OUTPUT);
  pinMode(COOLANT_PUMP_RELAY, OUTPUT);
  pinMode(AMBIENT_FAN_RELAY, OUTPUT);
  auto* coolant_fan_switch = new DigitalOutput(COOLANT_FAN_RELAY);
  auto* coolant_pump_switch = new DigitalOutput(COOLANT_PUMP_RELAY);
  auto* ambient_fan_switch = new DigitalOutput(AMBIENT_FAN_RELAY);
  // Create a switch controller to handle the user press logic and
  // connect it to the load switch...
  SmartSwitchController* coolant_fan_controller = new SmartSwitchController();
  SmartSwitchController* coolant_pump_controller = new SmartSwitchController();
  SmartSwitchController* ambient_fan_controller = new SmartSwitchController();
  coolant_fan_controller->connect_to(coolant_fan_switch);
  coolant_pump_controller->connect_to(coolant_pump_switch);
  ambient_fan_controller->connect_to(ambient_fan_switch);
  // Connect a physical button that will feed manual click types into the
  // controller.
  DigitalInputState* coolant_fan_btn =
      new DigitalInputState(COOLANT_FAN_BUTTON, INPUT_PULLDOWN, 100);
  DigitalInputState* coolant_pump_btn =
      new DigitalInputState(COOLANT_PUMP_BUTTON, INPUT_PULLDOWN, 100);
  DigitalInputState* ambient_fan_btn =
      new DigitalInputState(AMBIENT_FAN_BUTTON, INPUT_PULLDOWN, 100);
  PressRepeater* coolant_fan_pr = new PressRepeater();
  PressRepeater* coolant_pump_pr = new PressRepeater();
  PressRepeater* ambient_fan_pr = new PressRepeater();
  coolant_fan_btn->connect_to(coolant_fan_pr);
  coolant_pump_btn->connect_to(coolant_pump_pr);
  ambient_fan_btn->connect_to(ambient_fan_pr);
  coolant_fan_pr->connect_to(new ClickType(config_path_button_c))
      ->connect_to(coolant_fan_controller);
  coolant_pump_pr->connect_to(new ClickType(config_path_button_c))
      ->connect_to(coolant_pump_controller);
  ambient_fan_pr->connect_to(new ClickType(config_path_button_c))
      ->connect_to(ambient_fan_controller);
  // In addition to the manual button "click types", a
  // SmartSwitchController accepts explicit state settings via
  // any boolean producer as well as any "truth" values in human readable
  // format via a String producer.
  // Here, we set up a SignalK PUT request listener to handle
  // requests made to the Signal K server to set the switch state.
  // This allows any device on the SignalK network that can make
  // such a request to also control the state of our switch.
  auto* sk_listener_coolant_fan =
      new StringSKPutRequestListener(sk_path_coolant_fan);
  sk_listener_coolant_fan->connect_to(coolant_fan_controller);

  auto* sk_listener_coolant_pump =
      new StringSKPutRequestListener(sk_path_coolant_pump);
  sk_listener_coolant_pump->connect_to(coolant_pump_controller);

  auto* sk_listener_ambient_fan =
      new StringSKPutRequestListener(sk_path_ambient_fan);
  sk_listener_ambient_fan->connect_to(ambient_fan_controller);
  // Finally, connect the load switch to an SKOutput so it reports its state
  // to the Signal K server.  Since the load switch only reports its state
  // whenever it changes (and switches like light switches change infrequently),
  // send it through a `RepeatReport` transform, which will cause the state
  // to be reported to the server every 10 seconds, regardless of whether
  // or not it has changed.  That keeps the value on the server fresh and
  // lets the server know the switch is still alive.
  coolant_fan_switch
      ->connect_to(new RepeatReport<bool>(10000, config_path_repeat))
      ->connect_to(new SKOutputBool(sk_path_coolant_fan,
                                    config_path_sk_output_coolant_fan));

  coolant_pump_switch
      ->connect_to(new RepeatReport<bool>(10000, config_path_repeat))
      ->connect_to(new SKOutputBool(sk_path_coolant_pump,
                                    config_path_sk_output_coolant_pmp));

  ambient_fan_switch
      ->connect_to(new RepeatReport<bool>(10000, config_path_repeat))
      ->connect_to(new SKOutputBool(sk_path_ambient_fan,
                                    config_path_sk_output_ambient_fan));
  // -------------------------------------------------------------
  // SHAFT RPM
  // -------------------------------------------------------------
  sDisplayScale_t displayScale;
  displayScale.lowerDisplayScale = -40;
  displayScale.upperDisplayScale = 120;
  displayScale.typeDisplayScale = "linear";
  const uint8_t kRpmProxyInputPin = 23;
  pinMode(kRpmProxyInputPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(kRpmProxyInputPin), isr, RISING);
  auto* rpm_sensor = new RepeatSensor<float>(500, []() { return (shaftHz); });
  const char* sk_path = "propulsion.main.revolutions";
  auto rpm_sensor_metadata = new SKMetadata(
      "Hz",                               // units
      "Propeller Shaft RPM",              // display name
      "Propeller Shaft RPM",              // description
      "Shaft RPM",                        // short name
      10.0,                               // timeout, in seconds
      displayScale, 
      {"visual", "sound"},  // alert method (visual or sound)
      {"visual", "sound"},                // warn method (visual or sound)
      {"visual", "sound"},                // alarm method (visual or sound)
      {"visual", "sound"}                 // emergency method (visual or sound)
  );
  rpm_sensor->connect_to(new SKOutput<float>(
      sk_path, "/1_sensors/engine_rpm/sk", rpm_sensor_metadata));
  // -------------------------------------------------------------
  // Temperature sensors setup
  // -------------------------------------------------------------
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);
  // connect the 6 sensors, they have configuration data in the WebUI
  auto* temp_sensor_portMotor =
      new OneWireTemperature(dts, 2000, "/2_oneWire/portMotorTemp");
  auto* temp_sensor_starboardMotor =
      new OneWireTemperature(dts, 2000, "/2_oneWire/starboardMotorTemp");
  auto* temp_sensor_portController =
      new OneWireTemperature(dts, 2000, "/2_oneWire/portControllerTemp");
  auto* temp_sensor_starboardController =
      new OneWireTemperature(dts, 2000, "/2_oneWire/starboardControllerTemp");
  auto* temp_sensor_engineRoom =
      new OneWireTemperature(dts, 2000, "/2_oneWire/EngineRoomTemp");
  auto* temp_sensor_coolant =
      new OneWireTemperature(dts, 2000, "/2_oneWire/CoolantTemp");
  // Create the SK metadata for each sensor
  auto temp_sensor_portMotor_metadata = new SKMetadata(
      "K",                                // units
      "Port Motor Temperature",           // display name
      "Port Motor Temperature",           // description
      "Prt Mtr Tmp",                      // short name
      10.0,                               // timeout, in seconds
      displayScale, 
      {"visual", "sound"},  // alert method (visual or sound)
      {"visual", "sound"},                // warn method (visual or sound)
      {"visual", "sound"},                // alarm method (visual or sound)
      {"visual", "sound"}                 // emergency method (visual or sound)
  );
  auto temp_sensor_starboardMotor_metadata = new SKMetadata(
      "K",                                // units
      "Starboard Motor Temperature",      // display name
      "Starboard Motor Temperature",      // description
      "Sbrd Mtr Tmp",                     // short name
      10.0,                               // timeout, in seconds
      displayScale, 
      {"visual", "sound"},  // alert method (visual or sound)
      {"visual", "sound"},                // warn method (visual or sound)
      {"visual", "sound"},                // alarm method (visual or sound)
      {"visual", "sound"}                 // emergency method (visual or sound)
  );
  auto temp_sensor_portController_metadata = new SKMetadata(
      "K",                                // units
      "Port Controller Temperature",      // display name
      "Port Controller Temperature",      // description
      "Prt Cntrl Tmp",                    // short name
      10.0,                               // timeout, in seconds
      displayScale, 
      {"visual", "sound"},  // alert method (visual or sound)
      {"visual", "sound"},                // warn method (visual or sound)
      {"visual", "sound"},                // alarm method (visual or sound)
      {"visual", "sound"}                 // emergency method (visual or sound)
  );
  auto temp_sensor_starboardController_metadata = new SKMetadata(
      "K",                                 // units
      "Starboard Controller Temperature",  // display name
      "Starboard Controller Temperature",  // description
      "Sbrd Cntrl Tmp",                    // short name
      10.0,                                // timeout, in seconds
      displayScale, 
      {"visual", "sound"},   // alert method (visual or sound)
      {"visual", "sound"},                 // warn method (visual or sound)
      {"visual", "sound"},                 // alarm method (visual or sound)
      {"visual", "sound"}                  // emergency method (visual or sound)
  );
  auto temp_sensor_engineRoom_metadata = new SKMetadata(
      "K",                                // units
      "Engineroom Temperature",           // display name
      "Engineroom Temperature",           // description
      "Eng Rm Tmp",                       // short name
      10.0,                               // timeout, in seconds
      displayScale, 
      {"visual", "sound"},  // alert method (visual or sound)
      {"visual", "sound"},                // warn method (visual or sound)
      {"visual", "sound"},                // alarm method (visual or sound)
      {"visual", "sound"}                 // emergency method (visual or sound)
  );
  auto temp_sensor_coolant_metadata = new SKMetadata(
      "K",                                // units
      "Coolant Temperatur",               // display name
      "Coolant Temperature",              // description
      "Coolnt Tmp",                       // short name
      10.0,                               // timeout, in seconds
      displayScale, 
      {"visual", "sound"},  // alert method (visual or sound)
      {"visual", "sound"},                // warn method (visual or sound)
      {"visual", "sound"},                // alarm method (visual or sound)
      {"visual", "sound"}                 // emergency method (visual or sound)
  );
  // Connect the SK output paths
  temp_sensor_portMotor->connect_to(new SKOutput<float>(
      "propulsion.portMotor.temperature", "/2_oneWire/portMotorTemp/sk",
      temp_sensor_portMotor_metadata));
  temp_sensor_starboardMotor->connect_to(new SKOutput<float>(
      "propulsion.starboardMotor.temperature",
      "/2_oneWire/starboardMotorTemp/sk", temp_sensor_starboardMotor_metadata));
  temp_sensor_portController->connect_to(new SKOutput<float>(
      "propulsion.portController.temperature",
      "/2_oneWire/portControllerTemp/sk", temp_sensor_portController_metadata));
  temp_sensor_starboardController->connect_to(
      new SKOutput<float>("propulsion.starboardController.temperature",
                          "/2_oneWire/starboardControllerTemp/sk",
                          temp_sensor_starboardController_metadata));
  temp_sensor_engineRoom->connect_to(new SKOutput<float>(
      "propulsion.engineRoom.temperature", "/2_oneWire/EngineRoomTemp/sk",
      temp_sensor_engineRoom_metadata));
  temp_sensor_coolant->connect_to(new SKOutput<float>(
      "propulsion.coolant.temperature", "/2_oneWire/CoolantTemp/sk",
      temp_sensor_coolant_metadata));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() {
  app.tick();
  if (updateRpm) {
    // The ISR just save the micros at the occurance of the pulse
    // This code will check the difference between the previous pulse and the
    timeDifference = currentTime - lastTime;
    if (timeDifference > 0) {
      shaftHz = (1000000.0 / float(timeDifference));
    }
    // Update lastTime for the next pulse
    lastTime = currentTime;
    updateRpm = false;
  }
}
