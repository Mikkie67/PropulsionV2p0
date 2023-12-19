// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

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

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Kerosheba-Propulsion")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->enable_ota("Regurk67!")
                    ->get_app();

  // GPIO number to use for the analog input
  const uint8_t kAnalogInputPin = 36;
  // Define how often (in milliseconds) new samples are acquired
  const unsigned int kAnalogInputReadInterval = 500;
  // Define the produced value at the maximum input voltage (3.3V).
  // A value of 3.3 gives output equal to the input voltage.
  const float kAnalogInputScale = 3.3;

  // Create a new Analog Input Sensor that reads an analog input pin
  // periodically.
  auto* analog_input = new AnalogInput(
      kAnalogInputPin, kAnalogInputReadInterval, "", kAnalogInputScale);

  // Add an observer that prints out the current value of the analog input
  // every time it changes.
  analog_input->attach([analog_input]() {
    debugD("Analog input value: %f", analog_input->get());
  });

  // Set GPIO pin 15 to output and toggle it every 50 ms

  const uint8_t kDigitalOutputPin = 22;
  const unsigned int kDigitalOutputInterval =
      20;  // ridingh edge every 2ms, thus RPM of 500Hz
  pinMode(kDigitalOutputPin, OUTPUT);
  app.onRepeat(kDigitalOutputInterval, [kDigitalOutputPin]() {
    digitalWrite(kDigitalOutputPin, !digitalRead(kDigitalOutputPin));
  });

  // Read GPIO 14 every time it changes

  const uint8_t kDigitalInput1Pin = 14;
  auto* digital_input1 =
      new DigitalInputChange(kDigitalInput1Pin, INPUT_PULLUP, CHANGE);

  // Connect the digital input to a lambda consumer that prints out the
  // value every time it changes.

  // Test this yourself by connecting pin 15 to pin 14 with a jumper wire and
  // see if the value changes!

  digital_input1->connect_to(new LambdaConsumer<bool>(
      [](bool input) { debugD("Digital input value changed: %d", input); }));

  // Create another digital input, this time with RepeatSensor. This approach
  // can be used to connect external sensor library to SensESP!

  const uint8_t kDigitalInput2Pin = 13;
  const unsigned int kDigitalInput2Interval = 1000;

  // Configure the pin. Replace this with your custom library initialization
  // code!
  pinMode(kDigitalInput2Pin, INPUT_PULLUP);

  // Define a new RepeatSensor that reads the pin every 100 ms.
  // Replace the lambda function internals with the input routine of your custom
  // library.
  // Again, test this yourself by connecting pin 15 to pin 13 with a
  // jumper wire and see if the value changes!

  auto* digital_input2 = new RepeatSensor<bool>(
      kDigitalInput2Interval,
      [kDigitalInput2Pin]() { return digitalRead(kDigitalInput2Pin); });

  // Connect the analog input to Signal K output. This will publish the
  // analog input value to the Signal K server every time it changes.
  // analog_input->connect_to(new SKOutputFloat(
  //     "sensors.analog_input.voltage",         // Signal K path
  //     "/sensors/analog_input/voltage",        // configuration path, used in
  //     the
  //                                             // web UI and for storing the
  //                                             // configuration
  //     new SKMetadata("V",                     // Define output units
  //                    "Analog input voltage")  // Value description
  //     ));

  // // Connect digital input 2 to Signal K output.
  // digital_input2->connect_to(new SKOutputBool(
  //     "sensors.digital_input2.value",          // Signal K path
  //     "/sensors/digital_input2/value",         // configuration path
  //     new SKMetadata("",                       // No units for boolean values
  //                    "Digital input 2 value")  // Value description
  //     ));

  // -------------------------------------------------------------
  // KEROSHEBA PROPULSION ITEMS
  // -------------------------------------------------------------
  // SHAFT RPM
  // -------------------------------------------------------------
  const uint8_t kRpmProxyInputPin = 23;
  const char* sk_path = "propulsion.main.revolutions";
  // to count pulses and reports the readings every read_delay ms
  // (500 in the example). A Frequency
  // transform takes a number of pulses and converts that into
  // a frequency. The sample multiplier converts the 97 tooth
  // tach output into Hz, SK native units.
  const float multiplier = 1.0 / 1.0;
  const unsigned int read_delay = 500;

  // Wire it all up by connecting the producer directly to the consumer
  // ESP32 pins are specified as just the X in GPIOX
  auto* rpm_sensor = new DigitalInputCounter(kRpmProxyInputPin, INPUT_PULLUP,
                                             RISING, read_delay);
   auto rpm_sensor_metadata =
      new SKMetadata("Hz",                            // units
                     "Propeller Shaft RPM",  // display name
                     "Propeller Shaft RPM",  // description
                     "Shaft RPM",                 // short name
                     10.                             // timeout, in seconds
      );
rpm_sensor
      ->connect_to(new Frequency(
          multiplier, "/1_sensors/engine_rpm/calibrate"))  // connect the output of sensor
                                               // to the input of Frequency()
      ->connect_to(new SKOutputFloat(
          sk_path, "/1_sensors/engine_rpm/sk",rpm_sensor_metadata));  // connect the output of Frequency()
                                          // to a Signal K Output as a number
  // Temperature sensors setup
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);
  // connect the 6 sensors, they have configuration data in the WebUI
  auto* temp_sensor_portMotor =
      new OneWireTemperature(dts, 1000, "/2_oneWire/portMotorTemp");
  auto* temp_sensor_starboardMotor =
      new OneWireTemperature(dts, 1000, "/2_oneWire/starboardMotorTemp");
  auto* temp_sensor_portController =
      new OneWireTemperature(dts, 1000, "/2_oneWire/portControllerTemp");
  auto* temp_sensor_starboardController =
      new OneWireTemperature(dts, 1000, "/2_oneWire/starboardControllerTemp");
  auto* temp_sensor_engineRoom =
      new OneWireTemperature(dts, 1000, "/2_oneWire/EngineRoomTemp");
  auto* temp_sensor_coolant =
      new OneWireTemperature(dts, 1000, "/2_oneWire/CoolantTemp");
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
  temp_sensor_portMotor->connect_to(
      new SKOutput<float>("propulsion.portMotor.temperature","/2_oneWire/portMotorTemp/sk",temp_sensor_portMotor_metadata));
  temp_sensor_starboardMotor->connect_to(
      new SKOutput<float>("propulsion.starboardMotor.temperature","/2_oneWire/starboardMotorTemp/sk",temp_sensor_starboardMotor_metadata));
  temp_sensor_portController->connect_to(
      new SKOutput<float>("propulsion.portController.temperature","/2_oneWire/portControllerTemp/sk",temp_sensor_portController_metadata));
  temp_sensor_starboardController->connect_to(
      new SKOutput<float>("propulsion.starbaordController.temperature","/2_oneWire/starboardControllerTemp/sk",temp_sensor_starboardController_metadata));
  temp_sensor_engineRoom->connect_to(
      new SKOutput<float>("propulsion.engineRoom.temperature","/2_oneWire/EngineRoomTemp/sk",temp_sensor_engineRoom_metadata));
  temp_sensor_coolant->connect_to(
      new SKOutput<float>("propulsion.coolant.temperature","/2_oneWire/CoolantTemp/sk",temp_sensor_coolant_metadata));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
