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

#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"

using namespace sensesp;
// 1-Wire data pin on SH-ESP32
// ESP32 pins are specified as just the X in GPIOX
#define ONEWIRE_PIN 15
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
  // SHAFT RPM
  // -------------------------------------------------------------
  const uint8_t kRpmProxyInputPin = 23;
  pinMode(kRpmProxyInputPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(kRpmProxyInputPin), isr, RISING);
  auto* rpm_sensor = new RepeatSensor<float>(500, []() { return (shaftHz); });
  const char* sk_path = "propulsion.main.revolutions";
  auto rpm_sensor_metadata =
      new SKMetadata("Hz",                   // units
                     "Propeller Shaft RPM",  // display name
                     "Propeller Shaft RPM",  // description
                     "Shaft RPM",            // short name
                     10.0                    // timeout, in seconds
                    );
   rpm_sensor->connect_to(new SKOutput<float>(sk_path,"/1_sensors/engine_rpm/sk",rpm_sensor_metadata));
  // = new
  // SKOutput<float>(sk_path,"/1_sensors/engine_rpm/sk",rpm_sensor_metadata);
   //rpm_sensor_sk->connect_to(rpm_sensor);
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
  auto temp_sensor_portMotor_metadata =
      new SKMetadata("K",                       // units
                     "Port Motor Temperature",  // display name
                     "Port Motor Temperature",  // description
                     "Prt Mtr Tmp",             // short name
                     10.                        // timeout, in seconds
      );
  auto temp_sensor_starboardMotor_metadata =
      new SKMetadata("K",                            // units
                     "Starboard Motor Temperature",  // display name
                     "Starboard Motor Temperature",  // description
                     "Sbrd Mtr Tmp",                 // short name
                     10.                             // timeout, in seconds
      );
  auto temp_sensor_portController_metadata =
      new SKMetadata("K",                            // units
                     "Port Controller Temperature",  // display name
                     "Port Controller Temperature",  // description
                     "Prt Cntrl Tmp",                // short name
                     10.                             // timeout, in seconds
      );
  auto temp_sensor_starboardController_metadata =
      new SKMetadata("K",                                 // units
                     "Starboard Controller Temperature",  // display name
                     "Starboard Controller Temperature",  // description
                     "Sbrd Cntrl Tmp",                    // short name
                     10.                                  // timeout, in seconds
      );
  auto temp_sensor_engineRoom_metadata =
      new SKMetadata("K",                       // units
                     "Engineroom Temperature",  // display name
                     "Engineroom Temperature",  // description
                     "Eng Rm Tmp",              // short name
                     10.                        // timeout, in seconds
      );
  auto temp_sensor_coolant_metadata =
      new SKMetadata("K",                    // units
                     "Coolant Temperature",  // display name
                     "Coolant Temperature",  // description
                     "Coolnt Tmp",           // short name
                     10.                     // timeout, in seconds
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
