// Kerosheba Propulsion boat controller (VCU)

// #define SERIAL_DEBUG_DISABLED
// #define OTA_ENABLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#include "ESP32-TWAI-CAN.hpp"
#include "rpm_class.hpp"
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
#include "sensesp/transforms/click_type.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/press_repeater.h"
#include "sensesp/transforms/repeat_report.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"

using namespace sensesp;
// IOs on ESP32 DEVKIT
// D13
// D12
// D14
// D27  SDA_PIN
// D26  SCL_PIN
// D25
// D33
// D32  COOLANT_FAN_RELAY
// D35  COOLANT_PUMP_RELAY
// D34  AMBIENT_FAN_RELAY
// --Other side--
// D15  ONEWIRE_PIN
// D2
// D4   CAN_TX
// RX2
// TX2
// D5   CAN_RX
// D18  COOLANT_FAN_BUTTON
// D19  COOLANT_PUMP_BUTTON
// D21  AMBIENT_FAN_BUTTON
// RXD
// TXD
// D22  CAN_CLK
// D23  CAN_BUS_OFF
// DISPLAY defines
#define SDA_PIN 27
#define SCL_PIN 26
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// 1-Wire data pin
#define ONEWIRE_PIN 15
// Button inputs to manually control fans and pumps
#define COOLANT_FAN_BUTTON 18
#define COOLANT_PUMP_BUTTON 19
#define AMBIENT_FAN_BUTTON 21
// Outputs to control relays for fans and pumps
#define COOLANT_FAN_RELAY 32
#define COOLANT_PUMP_RELAY 33
#define AMBIENT_FAN_RELAY 34
// CAN defines
#define CAN_TX 4
#define CAN_RX 5
#define CAN_CLK 22
#define CAN_BUS_OFF 23
typedef enum {
  STATE_CLOSED = 0,
  STATE_CONNECTED = 1,
  STATE_SYNCED = 2,
  STATE_MSG1 = 3,
  STATE_MSG2 = 4,
  STATE_LAST = 5
} eCanFsmState_t;
char State[6][8] = {"CLOSED\0", "OPEN\0",   "SYNCED\0",
                    "MSG1  \0", "MSG2  \0", "LAST  \0"};
eCanFsmState_t sCurrentCan0State = STATE_CLOSED;
uint32_t CanRxMsg = 0;
uint32_t CanTxMsg = 0;
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
CanFrame rxFrame;
TwoWire *i2c;
Adafruit_SSD1306 *display;
CanFrame txFrame;
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

// ----------------------------------------------------------
// CAN SYNC REPLY
// ----------------------------------------------------------
uint8_t LifeSignal = 0;
uint8_t GetSA(CanFrame aCanFrame) {
  uint32_t lID = aCanFrame.identifier;
  return (uint8_t)lID;
}
uint8_t GetPS1(CanFrame aCanFrame) {
  uint32_t lID = aCanFrame.identifier & 0x0000FF00;
  return (uint8_t)lID >> 8;
}
uint8_t GetPF1(CanFrame aCanFrame) {
  uint32_t lID = aCanFrame.identifier & 0x00FF0000;
  return (uint8_t)lID >> 16;
}

bool CAN_SendSyncReply(uint8_t McuId) {
  bool bRet = false;
  CanFrame SyncFrame = {0};
  SyncFrame.identifier = 0x0C01EFD0;
  SyncFrame.extd = 1;
  SyncFrame.data_length_code = 8;
  SyncFrame.data[0] = 0xAA;
  SyncFrame.data[1] = 0xAA;
  SyncFrame.data[2] = 0xAA;
  SyncFrame.data[3] = 0xAA;
  SyncFrame.data[4] = 0xAA;
  SyncFrame.data[5] = 0xAA;
  SyncFrame.data[6] = 0xAA;
  SyncFrame.data[7] = 0xAA;
  bRet = ESP32Can.writeFrame(SyncFrame);  // timeout defaults to 1 ms
  if (bRet) {
    CanTxMsg++;
  }
  return bRet;
}
typedef struct {
  int16_t BusVoltage;
  int16_t BusCurrent;
  int16_t PhaseCurrent;
  int16_t SpeedRpm;
  int16_t ControllerTemp;
  int16_t MotorTemp;
  int16_t ThrottlePosition;
  uint8_t Status;
  uint32_t Error;
  uint8_t LifeSignal;
} sMcuData_t;
sMcuData_t sMcuData;
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
  // CAN BUS
  Serial.printf("CAN Init Status %d", ESP32Can.getInit());
  ESP32Can.setPins(CAN_TX, CAN_RX, CAN_CLK, CAN_BUS_OFF);
  ESP32Can.setRxQueueSize(8);
  ESP32Can.setTxQueueSize(8);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(250));
  if (ESP32Can.begin()) {
    Serial.println("CAN bus started!");
    sCurrentCan0State = STATE_CONNECTED;
  } else {
    Serial.println("CAN bus failed!");
    sCurrentCan0State = STATE_CLOSED;
  }
  // // Check the CAN bus receiver every 1ms
  app.onRepeat(1, []() 
  {
    if (ESP32Can.readFrame(&rxFrame, 1000)) 
    {
      Serial.printf("Received frame: %08X  \r\n", rxFrame.identifier);
      // If Frame is from MCU, check if it is Sync Frame, if so, reply
      // accordingly, otherwise
      //  check the frame for the information and populate structures from data
      if (GetSA(rxFrame) == 239)  // TODO change to variable nodeid for two
                                  // different MCUs (via config page)
      {
        Serial.println("CAN frame received");
        // Check if it is a handshake message
        if ((rxFrame.data[0] == 0x55) && (rxFrame.data[1] == 0x55) &&
            (rxFrame.data[2] == 0x55) && (rxFrame.data[3] == 0x55) &&
            (rxFrame.data[4] == 0x55) && (rxFrame.data[5] == 0x55) &&
            (rxFrame.data[6] == 0x55) && (rxFrame.data[7] == 0x55)) {
          CAN_SendSyncReply(239);
          sCurrentCan0State = STATE_SYNCED;
        }
      }
    } else
    {
      // do nothing, frame not meant for this node
    }
    // data message from MCU, populate structures
    // Message I
    if (GetPF1(rxFrame) == 0x01) {
      sCurrentCan0State = STATE_MSG1;
      sMcuData.BusVoltage = (rxFrame.data[0] << 8 + rxFrame.data[1]) * 0.1;
      sMcuData.BusCurrent = (rxFrame.data[2] << 8 + rxFrame.data[3]) * 0.1;
      sMcuData.PhaseCurrent = (rxFrame.data[4] << 8 + rxFrame.data[5]) * 0.1;
      sMcuData.SpeedRpm = (rxFrame.data[6] << 8 + rxFrame.data[7]) * 0.1;
    }
    // Message II
    if (GetPF1(rxFrame) == 0x02) {
      sCurrentCan0State = STATE_MSG2;
      sMcuData.ControllerTemp = (rxFrame.data[0]);
      sMcuData.MotorTemp = (rxFrame.data[1]);
      sMcuData.ThrottlePosition = (rxFrame.data[2]);
      sMcuData.Status = (rxFrame.data[3]);
      sMcuData.Error =
          (rxFrame.data[4] << 24 + rxFrame.data[5] << 16 + rxFrame.data[6]
                           << 8 + rxFrame.data[7]) >>
          4;
      sMcuData.LifeSignal = (rxFrame.data[7] & 0x0F);
    }
});
//   Send the CAN control command when in a state later than synced
app.onRepeat(50, []() {

 // if (sCurrentCan0State > STATE_SYNCED) 
  {
    int16_t TargetPhaseCurrent = 50;
    int16_t TargetSpeed = 1000;
    uint8_t ControlMode = 0x03;  // running in speed control mode
    CanFrame TxFrame = {0};
    TxFrame.identifier = 0x0C01EFD0;
    TxFrame.extd = 1;
    TxFrame.data_length_code = 8;
    TxFrame.data[0] = uint8_t(((uint16_t)TargetPhaseCurrent));
    TxFrame.data[1] = uint8_t(((uint16_t)TargetPhaseCurrent) >> 8);
    TxFrame.data[2] = uint8_t(((uint16_t)TargetSpeed));
    TxFrame.data[3] = uint8_t(((uint16_t)TargetSpeed) >> 8);
    TxFrame.data[4] = ControlMode;
    TxFrame.data[5] = 0xAA;  // to avoid bit-stuffing
    TxFrame.data[6] = 0xAA;
    TxFrame.data[7] = LifeSignal;
    // Accepts both pointers and references
    ESP32Can.writeFrame(TxFrame);  // timeout defaults to 1 ms
    LifeSignal++;
  }
});

/*
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
  pinMode(COOLANT_FAN_RELAY, OUTPUT);
  pinMode(COOLANT_PUMP_RELAY, OUTPUT);
  pinMode(AMBIENT_FAN_RELAY, OUTPUT);
  auto* coolant_fan_switch = new DigitalOutput(COOLANT_FAN_RELAY);
  auto* coolant_pump_switch = new DigitalOutput(COOLANT_PUMP_RELAY);
  auto* ambient_fan_switch = new DigitalOutput(AMBIENT_FAN_RELAY);
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
  // whenever it changes (and switches like light switches change
  infrequently),
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
  // Dispaly scale meta configuration
  sDisplayScale_t displayScale;
  displayScale.lowerDisplayScale = 0;
  displayScale.upperDisplayScale = 1500;
  displayScale.typeDisplayScale = "linear";
  // zones meta information for RPM (hardcoded to 4 but some can be left
  blank)
  // nominal   : this is a special type of normal state/zone (see below)
  // normal    : the normal operating range for the value in question
  (default)
  // alert	   : Indicates a safe or normal condition which is brought to
  // the operators attention to impart information for routine action purposes
  // warn	     : Indicates a condition that requires immediate attention
  // but not immediate action alarm	   : Indicates a condition which is
  // outside the specified acceptable range. Immediate action is required to
  // prevent loss of life or equipment damage emergency : the value indicates
  a
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
                     {"visual", "sound"},  // emergency method (visual or
  sound) zonesRpm              // zones array of zone structs
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
  // zones meta information for Temp (hardcoded to 4 but some can be left
  blank) std::array<sZone_t, 4> zonesTemp; zonesTemp.at(0).lowerZone = -40;
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
      {"visual", "sound"},                // emergency method (visual or
  sound) zonesTemp                           // zones array of zone structs
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
      {"visual", "sound"},                // emergency method (visual or
  sound) zonesTemp                           // zones array of zone structs
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
      {"visual", "sound"},                // emergency method (visual or
  sound) zonesTemp                           // zones array of zone  structs
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
      {"visual", "sound"},                 // emergency method (visual or
  sound) zonesTemp                            // zones array of zone  structs
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
      {"visual", "sound"},                // emergency method (visual or
  sound) zonesTemp                           // zones array of zone  structs
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
      {"visual", "sound"},                // emergency method (visual or
  sound) zonesTemp                           // zones array of zone structs
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
  */
// initialize the display
i2c = new TwoWire(0);
i2c->begin(SDA_PIN, SCL_PIN);
display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
  Serial.println(F("SSD1306 allocation failed"));
}
delay(100);
display->setRotation(2);
display->clearDisplay();
display->display();

// update results
app.onRepeat(10, []() {
  display->clearDisplay();
  display->setTextSize(1);
  display->setCursor(0, 0);
  display->setTextColor(SSD1306_WHITE);
  display->printf("KEROSHEBA MOTOR CNTRL\n");
  display->printf("CAN: %d Uptime: %d\n", sCurrentCan0State, millis() / 1000);
  display->printf("RX: %d    TX: %02d: %d\n", CanRxMsg, LifeSignal, CanTxMsg);
  // display->printf("123456789012345678901\n");
  display->printf("Bus V  Bus I  Phase I");
  display->printf("%2.2f %#6d %#7d\n", 54.2, 123, 245);
  display->printf("Speed RPM: %#5d\n", -2300);
  display->printf("CT:%#2.1f MT:%#2.1f %#2d\n", -20.1, 23.4, 52);
  display->display();
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
