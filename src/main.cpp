// Kerosheba Propulsion boat controller (VCU)

// TODO
//  fix the display
//  get throttle value from ADC
//  add modbus for battery packs
//  add UI elements for CAN bus
//       control mode
//       mcu ids
//       vcu id

// #define SERIAL_DEBUG_DISABLED
// #define OTA_ENABLED
#include "bms_parser.h"
#include "Adafruit_GFX.h"
#include "ILI9488.h"
#include "esp32ModbusRTU.h"
#include "ker_can.hpp"
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
#include "rpm_class.hpp"
#include "sensesp/transforms/click_type.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/press_repeater.h"
#include "sensesp/transforms/repeat_report.h"

using namespace sensesp;
unsigned long testFillScreen();
unsigned long testText();
unsigned long testLines(uint16_t color);
unsigned long testFastLines(uint16_t color1, uint16_t color2);
unsigned long testRects(uint16_t color);
unsigned long testFilledRects(uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(uint8_t radius, uint16_t color);
unsigned long testCircles(uint8_t radius, uint16_t color);
unsigned long testTriangles();
unsigned long testFilledTriangles();
unsigned long testRoundRects();
unsigned long testFilledRoundRects();
// -----------------------------------------------------------------------------
// DISPLAY defines
// -----------------------------------------------------------------------------
// #define SDA_PIN 27
// #define SCL_PIN 26
// -----------------------------------------------------------------------------
// 1-Wire data pin
// -----------------------------------------------------------------------------
const byte OneWirePin = 37; //KerProp.1Wire
// -----------------------------------------------------------------------------
// Opto inputs for status of fans and pumps
// -----------------------------------------------------------------------------
const byte  CoolantFanStatus  = 39; // KerProp.DigIn1
const byte  CoolantPumpStatus = 40; // KerProp.DigIn2
const byte  AmbientFanStatus  = 41; // KerProp.DigIn3
const byte  SpareOptoInput    = 42; // KerProp.DigIn4
// -----------------------------------------------------------------------------
// Opto output 
// -----------------------------------------------------------------------------
const byte  SpareOptoOutput1 = 45; // KerProp.DigOut1
const byte  SpareOptoOutput2 = 46; // KerProp.DigOut2
const byte  SpareOptoOutput3 = 47; // KerProp.DigOut3
const byte  SpareOptoOutput4 = 48; // KerProp.DigOut4
// -----------------------------------------------------------------------------
// Solid State Relay outputs to control relays for fans and pumps
// -----------------------------------------------------------------------------
const byte  CoolantFanControl  = 9;  // KerPRop.RelayControl1
const byte  CoolantPumpControl = 10; // KerProp.RelayControl2
const byte  AmbientFanControl  = 11; // KerProp.RelayControl3
const byte  SpareControl1      = 12; // KerProp.RelayControl4
const byte  SpareControl2      = 13; // KerProp.RelayControl5
const byte  SpareControl3      = 14; // KerProp.RelayControl6
// -----------------------------------------------------------------------------
// MODBUS BMS defines --> using serial port 1
// -----------------------------------------------------------------------------
const byte ModBusDePin  = 8;  // KerProp.MOD_DE
const byte ModBusTxdPin = 17; // KerProp.MOD_TXD
const byte ModBusRxdPin = 18; // KerProp.MOD_RXD
esp32ModbusRTU BMS_modbus(&Serial1, ModBusTxdPin, ModBusRxdPin, ModBusDePin);
sBMS_data_t sBMS0_data;
sBMS_data_t sBMS1_data;
// -----------------------------------------------------------------------------
// CAN defines --> using TWAI controller
// -----------------------------------------------------------------------------
const byte  CAN_TX      = 16; // KerProp.CAN_TXD
const byte  CAN_RX      = 15; // KerProp.CAN_RXD
const byte  CAN_CLK     = -1;
const byte  CAN_BUS_OFF = -1;
ker_can myKerCan(239, 240);
// -----------------------------------------------------------------------------
//  Analog (Throttle)
// -----------------------------------------------------------------------------
const uint8_t kAnalogInputPin = 1; // KerProp.ADC_IN1
// Define how often (in milliseconds) new samples are acquired
const unsigned int kAnalogInputReadInterval = 500;
// Define the produced value at the maximum input voltage (3.3V).
// A value of 3.3 gives output of 100%
// TODO I will make use of a lookup table as well as interpolating in the table
const float kAnalogInputScale = 100;
// -----------------------------------------------------------------------------
//  LCD
// -----------------------------------------------------------------------------
const byte  LCD_CS     = 7;  // KerProp.LCD_CS
const byte  LCD_DC     = 6;  // KerProp.LCD_DC
const byte  LCD_MOSI   = 35; // KerProp.LCD_MOSI
const byte  LCD_CLK    = 0;  // KerProp.LCD_CLK
const byte  LCD_RST    = 5;  // KerProp.LCD_RST
const byte  LCD_MISO   = 36; // KerProp.LCD_MISO
const byte  LCD_LED    = 4;  // KerProp.LCD_LED

ILI9488 tft = ILI9488(LCD_CS, LCD_DC, LCD_MOSI, LCD_CLK, LCD_RST, LCD_MISO);


#define DEBUGGING_ENABLED
// -----------------------------------------------------------------------------
// Temp defines
float KelvinToCelsius(float temp) { return temp - 273.15; }
double portMotor_temperature = -128;
double portController_temperature = -128;
double starboardMotor_temperature = -128;
double starboardController_temperature = -128;
double engineroom_temperature = -128;
double coolant_temperature = -128;
ShaftFrequency clShaftFreq(0, 0.05);
reactesp::ReactESP app;
//TODO remove?? sDisplayData_t sDisplayData = {0};
uint8_t state_index = 0;

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
  lastTime = micros();
  updateRpm = true;
}
// display_ssd1306 *myDisplay;

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
                    ->set_hostname("Propulsion1")
// Optionally, hard-code the WiFi and Signal K server
// settings. This is normally not needed.
//->set_wifi("My WiFi SSID", "my_wifi_password")
//->set_sk_server("192.168.10.3", 80)
#ifdef OTA_ENABLED
                    ->enable_ota("Regurk67!")
#endif
                    ->get_app();

  // Set GPIO pin 22 to output and toggle it every 10 ms
  const uint8_t kDigitalOutputPin = 22;
  const unsigned int kDigitalOutputInterval = 20;
  pinMode(kDigitalOutputPin, OUTPUT);
  app.onRepeat(kDigitalOutputInterval, [kDigitalOutputPin]() {
    digitalWrite(kDigitalOutputPin, !digitalRead(kDigitalOutputPin));
  });

  // -------------------------------------------------------------
  // KEROSHEBA PROPULSION ITEMS
  // -------------------------------------------------------------
  // THROTTLE INPUT
  // -------------------------------------------------------------
  // Create a new Analog Input Sensor that reads an analog input pin
  // periodically.
  //   auto* Throttle = new AnalogInput(kAnalogInputPin,
  //   kAnalogInputReadInterval,
  //                                    "", kAnalogInputScale);
  //   // Add an observer that prints out the current value of the analog input
  //   // every time it changes.
  //   Throttle->attach(
  //       [Throttle]() { debugD("Throttle value: %#3.2f", Throttle->get()); });
  // -------------------------------------------------------------
  // LCD display
  // -------------------------------------------------------------
  digitalWrite(LCD_LED,1);
  digitalWrite(LCD_RST,0);
  tft.begin();
    // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9488_RDMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9488_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9488_RDPIXFMT);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9488_RDIMGFMT);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9488_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);

  Serial.println(F("Benchmark                Time (microseconds)"));

  Serial.print(F("Screen fill              "));
  Serial.println(testFillScreen());
  delay(500);

  Serial.print(F("Text                     "));
  Serial.println(testText());
  delay(3000);

  // Serial.print(F("Lines                    "));
  // Serial.println(testLines(ILI9488_CYAN));
  // delay(500);

  // Serial.print(F("Horiz/Vert Lines         "));
  // Serial.println(testFastLines(ILI9488_RED, ILI9488_BLUE));
  // delay(500);

  // Serial.print(F("Rectangles (outline)     "));
  // Serial.println(testRects(ILI9488_GREEN));
  // delay(500);

  // Serial.print(F("Rectangles (filled)      "));
  // Serial.println(testFilledRects(ILI9488_YELLOW, ILI9488_MAGENTA));
  // delay(500);

  // Serial.print(F("Circles (filled)         "));
  // Serial.println(testFilledCircles(10, ILI9488_MAGENTA));

  // Serial.print(F("Circles (outline)        "));
  // Serial.println(testCircles(10, ILI9488_WHITE));
  // delay(500);

  // Serial.print(F("Triangles (outline)      "));
  // Serial.println(testTriangles());
  // delay(500);

  // Serial.print(F("Triangles (filled)       "));
  // Serial.println(testFilledTriangles());
  // delay(500);

  // Serial.print(F("Rounded rects (outline)  "));
  // Serial.println(testRoundRects());
  // delay(500);

  // Serial.print(F("Rounded rects (filled)   "));
  // Serial.println(testFilledRoundRects());
  // delay(500);

  Serial.println(F("Done!"));
// -------------------------------------------------------------
  // CAN BUS
  // -------------------------------------------------------------
 //IVOR myKerCan.begin(CAN_TX, CAN_RX, CAN_CLK, CAN_BUS_OFF);
  // Check the CAN bus receiver every 1ms
//IVOR  app.onRepeat(1, []() { myKerCan.checkReceiver(); });
  //  Send the CAN control command when in a state later than synced
//IVOR  app.onRepeat(50, []() { myKerCan.SendCommands(120, 100, 3); });
  // -------------------------------------------------------------
  // MOD BUS for 2x BMS
  // -------------------------------------------------------------
  BMS_modbus.onData([](uint8_t serverAddress, esp32Modbus::FunctionCode fc,
                       uint16_t address, uint8_t* data, size_t length) {
    //.+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++Serial.printf("Addr = 0x%04X\n", address);
    switch (address) {
      case 0xD000: {
        if (serverAddress == 0x01) {
          Parse_D0000026(&sBMS0_data, data, length);
        } else if (serverAddress == 0x02) {
          Parse_D0000026(&sBMS1_data, data, length);
        }
      } break;
      case 0xD026: {
        if (serverAddress == 0x01) {
          Parse_D0260019(&sBMS0_data, data, length);
        } else if (serverAddress == 0x02) {
          Parse_D0260019(&sBMS1_data, data, length);
        }
      } break;
      case 0xD100: {
         if (serverAddress == 0x01) {
          Parse_D1000015(&sBMS0_data, data, length);
        } else if (serverAddress == 0x02) {
          Parse_D1000015(&sBMS1_data, data, length);
        }
     } break;
      case 0xD200: {
        if (serverAddress == 0x01) {
          Parse_D0260019(&sBMS0_data, data, length);
        } else if (serverAddress == 0x02) {
          Parse_D0260019(&sBMS1_data, data, length);
        }
      } break;
    }
  });
  BMS_modbus.onError([](esp32Modbus::Error error) {
    Serial.printf("error: 0x%02x\n\n", static_cast<uint8_t>(error));
  });
  Serial2.begin(19200, SERIAL_8N1, 16, 17);  // Modbus connection
  BMS_modbus.begin();
  app.onRepeat(500, []() {
    switch (state_index) {
      case 0: BMS_modbus.readHoldingRegisters(0x01, 0xD000, 0x0026); state_index++; break;
      case 1: BMS_modbus.readHoldingRegisters(0x01, 0xD026, 0x0019); state_index++; break;
      case 2: BMS_modbus.readHoldingRegisters(0x01, 0xD100, 0x0015); state_index++; break;
      case 3: BMS_modbus.readHoldingRegisters(0x01, 0xD200, 0x0001); state_index = 0; break;
    }
  });
  
    // -------------------------------------------------------------
    // COOLANT FAN, PUMP AND AMBIENT FAN (ENGINE ROOM FAN)
    // -------------------------------------------------------------
    const char* sk_path_coolant_fan = "propulsion.switches.coolant.fan.state";
    const char* sk_path_coolant_pump =
    "propulsion.switches.coolant.pump.state"; const char* sk_path_ambient_fan =
    "propulsion.switches.ambient.fan.state";
    // "Configuration paths" are combined with "/config" to formulate a URL
    // used by the RESTful API for retrieving or setting configuration data.
    // It is ALSO used to specify a path to the SPIFFS file system
    // where configuration data is saved on the MCU board.  It should
    // ALWAYS start with a forward slash if specified.  If left blank,
    // that indicates a sensor or transform does not have any
    // configuration to save.
    const char* config_path_button_c = "/button/clicktime";
    const char* config_path_status_light = "/button/statusLight";
    const char* config_path_sk_output_coolant_fan =
    "/signalk/path/coolant/fan"; const char* config_path_sk_output_coolant_pmp
    = "/signalk/path/coolant/pump"; const char*
    config_path_sk_output_ambient_fan = "/signalk/path/ambient/fan"; const
    char* config_path_repeat = "/signalk/repeat";

    // Create a digital output that is assumed to be connected to the
    // control channel of a relay or a MOSFET that will control the
    // coolant fan.
    pinMode(CoolantFanControl, OUTPUT);
    pinMode(CoolantPumpControl, OUTPUT);
    pinMode(AmbientFanControl, OUTPUT);
    auto* coolant_fan_switch = new DigitalOutput(CoolantFanControl);
    auto* coolant_pump_switch = new DigitalOutput(CoolantPumpControl);
    auto* ambient_fan_switch = new DigitalOutput(AmbientFanControl);
    // Create a switch controller to handle the user press logic and
    // connect it to the load switch...
    SmartSwitchController* coolant_fan_controller = new
    SmartSwitchController(); SmartSwitchController* coolant_pump_controller =
    new SmartSwitchController(); SmartSwitchController* ambient_fan_controller
    = new SmartSwitchController();
    coolant_fan_controller->connect_to(coolant_fan_switch);
    coolant_pump_controller->connect_to(coolant_pump_switch);
    ambient_fan_controller->connect_to(ambient_fan_switch);
    // Connect a physical button that will feed manual click types into the
    // controller.
    DigitalInputState* coolant_fan_status =
        new DigitalInputState(CoolantFanStatus, INPUT_PULLDOWN, 100);
    DigitalInputState* coolant_pump_status =
        new DigitalInputState(CoolantPumpStatus, INPUT_PULLDOWN, 100);
    DigitalInputState* ambient_fan_status =
        new DigitalInputState(AmbientFanStatus, INPUT_PULLDOWN, 100);
    PressRepeater* coolant_fan_pr = new PressRepeater();
    PressRepeater* coolant_pump_pr = new PressRepeater();
    PressRepeater* ambient_fan_pr = new PressRepeater();
    coolant_fan_status->connect_to(coolant_fan_pr);
    coolant_pump_status->connect_to(coolant_pump_pr);
    ambient_fan_status->connect_to(ambient_fan_pr);
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
    // whenever it changes (and switches like light switches change
    // infrequently),
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
    // Display scale meta configuration
    sDisplayScale_t displayScale;
    displayScale.lowerDisplayScale = 0;
    displayScale.upperDisplayScale = 1500;
    displayScale.typeDisplayScale = "linear";
    // zones meta information for RPM (hardcoded to 4 but some can be left
    // blank)
    // nominal   : this is a special type of normal state/zone (see below)
    // normal    : the normal operating range for the value in question
    // (default)
    // alert	   : Indicates a safe or normal condition which is brought to
    // the operators attention to impart information for routine action purposes
    // warn	     : Indicates a condition that requires immediate attention
    // but not immediate action alarm	   : Indicates a condition which is
    // outside the specified acceptable range. Immediate action is required to
    // prevent loss of life or equipment damage emergency : the value indicates
    // a
    // life-threatening condition
    std::array<sZone_t, 4> zonesRpm;
    zonesRpm.at(0).lowerZone = 0;
    zonesRpm.at(0).upperZone = 1000;
    zonesRpm.at(0).stateZone = "nominal";
    zonesRpm.at(0).messageZone = "Normal prop speed";
    zonesRpm.at(1).lowerZone = 1000;
    zonesRpm.at(1).upperZone = 1200;
    zonesRpm.at(1).stateZone = "warn";
    zonesRpm.at(1).messageZone = "High prop speed";
    zonesRpm.at(2).lowerZone = 1200;
    zonesRpm.at(2).upperZone = 1500;
    zonesRpm.at(2).stateZone = "alarm";
    zonesRpm.at(2).messageZone = "Maximum prop speed";
    const uint8_t kRpmProxyInputPin = 23;
    pinMode(kRpmProxyInputPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kRpmProxyInputPin), isr, RISING);
    auto* rpm_sensor = new RepeatSensor<float>(100, []() { return (shaftHz); });
    const char* sk_path = "propulsion.main.revolutions";
    auto rpm_sensor_metadata =
        new SKMetadata("Hz",                   // units
                       "Propeller Shaft RPM",  // display name
                       "Propeller Shaft RPM",  // description
                       "Shaft RPM",            // short name
                       10.0,                   // timeout, in seconds
                       displayScale,           // dsiplay scale structure
                       {"visual", "sound"},    // alert method (visual or sound)
                       {"visual", "sound"},    // warn method (visual or sound)
                       {"visual", "sound"},    // alarm method (visual or sound)
                       {"visual", "sound"},  // emergency method (visual or sound) 
                       zonesRpm              // zones array of zone structs
        );
    rpm_sensor->connect_to(new SKOutput<float>(
        sk_path, "/1_sensors/engine_rpm/sk", rpm_sensor_metadata));
    // -------------------------------------------------------------
    // Temperature sensors setup
    // -------------------------------------------------------------
    DallasTemperatureSensors* dts = new DallasTemperatureSensors(OneWirePin);
    // connect the 6 sensors, they have configuration data in the WebUI
    auto* temp_sensor_portMotor =
        new OneWireTemperature(dts, 2000, "/2_oneWire/portMotorTemp");
    auto* temp_sensor_starboardMotor =
        new OneWireTemperature(dts, 2000, "/2_oneWire/starboardMotorTemp");
    auto* temp_sensor_portController =
        new OneWireTemperature(dts, 2000, "/2_oneWire/portControllerTemp");
    auto* temp_sensor_starboardController =
        new OneWireTemperature(dts, 2000,
        "/2_oneWire/starboardControllerTemp");
    auto* temp_sensor_engineRoom =
        new OneWireTemperature(dts, 2000, "/2_oneWire/EngineRoomTemp");
    auto* temp_sensor_coolant =
        new OneWireTemperature(dts, 2000, "/2_oneWire/CoolantTemp");
    // Create the SK metadata for each sensor
    displayScale.lowerDisplayScale = -40;
    displayScale.upperDisplayScale = 120;
    displayScale.typeDisplayScale = "linear";
    // zones meta information for Temp (hardcoded to 4 but some can be left blank) 
    std::array<sZone_t, 4> zonesTemp; zonesTemp.at(0).lowerZone = -40;
    zonesTemp.at(0).upperZone = 60;
    zonesTemp.at(0).stateZone = "nominal";
    zonesTemp.at(0).messageZone = "Normal Temp";
    zonesTemp.at(1).lowerZone = 60;
    zonesTemp.at(1).upperZone = 80;
    zonesTemp.at(1).stateZone = "alert";
    zonesTemp.at(1).messageZone = "High Temp";
    zonesTemp.at(2).lowerZone = 80;
    zonesTemp.at(2).upperZone = 120;
    zonesTemp.at(2).stateZone = "alarm";
    zonesTemp.at(2).messageZone = "Maximum Temp";
    Serial.println(zonesTemp.at(0).messageZone);
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
        {"visual", "sound"},                // emergency method (visual or sound) 
        zonesTemp                           // zones array of zone structs
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
        {"visual", "sound"},                // emergency method (visual or sound) 
        zonesTemp                           // zones array of zone structs
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
        {"visual", "sound"},                // emergency method (visual or sound) 
        zonesTemp                           // zones array of zone  structs
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
        {"visual", "sound"},                 // emergency method (visual or sound) 
        zonesTemp                            // zones array of zone  structs
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
        {"visual", "sound"},                // emergency method (visual or sound) 
        zonesTemp                           // zones array of zone  structs
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
        {"visual", "sound"},                // emergency method (visual or sound) 
        zonesTemp                           // zones array of zone structs
    );
    // Connect the SK output paths
    temp_sensor_portMotor->connect_to(new SKOutput<float>(
        "propulsion.portMotor.temperature", "/2_oneWire/portMotorTemp/sk",
        temp_sensor_portMotor_metadata));
    temp_sensor_starboardMotor->connect_to(new SKOutput<float>(
        "propulsion.starboardMotor.temperature",
        "/2_oneWire/starboardMotorTemp/sk",
        temp_sensor_starboardMotor_metadata));
    temp_sensor_portController->connect_to(new SKOutput<float>(
        "propulsion.portController.temperature",
        "/2_oneWire/portControllerTemp/sk",
        temp_sensor_portController_metadata));
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

    app.onRepeat(100, [](){if ((int)shaftHz !=
    (int)clShaftFreq.getCurrentFrequency()){ shaftHz =
    clShaftFreq.getCurrentFrequency();}});
    
  // myDisplay->begin(SDA_PIN, SCL_PIN);
  //   update results
  app.onRepeat(10, []() {
    // TODO sDisplayData.BusCurrent = Can bus current;
    // myDisplay->updateDisplay(sDisplayData);
  });
  sensesp_app->start();
}
void loop() {
  app.tick();
  if (updateRpm) {
    clShaftFreq.pulseReceived(micros());
    updateRpm = false;
  } else {
    clShaftFreq.update(micros());
  }
}


unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9488_BLACK);
  tft.fillScreen(ILI9488_RED);
  tft.fillScreen(ILI9488_GREEN);
  tft.fillScreen(ILI9488_BLUE);
  tft.fillScreen(ILI9488_BLACK);
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(ILI9488_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9488_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9488_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9488_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9488_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(ILI9488_BLACK);

  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  tft.fillScreen(ILI9488_BLACK);

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  tft.fillScreen(ILI9488_BLACK);

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  tft.fillScreen(ILI9488_BLACK);

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);

  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(ILI9488_BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(ILI9488_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9488_BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(ILI9488_BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9488_BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(0, 0, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9488_BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i, i));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i, i, 0));
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9488_BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=6) {
    i2 = i / 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9488_BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()); i>20; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
  }

  return micros() - start;
}