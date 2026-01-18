// Kerosheba Propulsion boat controller (VCU)

// TODO
//  get throttle value from ADC
//  add UI elements for CAN bus
//       control mode
//       mcu ids
//       vcu id

// #define SERIAL_DEBUG_DISABLED
// #define OTA_ENABLED
#include "Adafruit_GFX.h"
#include "ILI9488.h"
#include "bms_parser.h"
#include "esp32ModbusRTU.h"
#include "ker_can.hpp"
#include "propDisplay.h"
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
void setupLcdDisplay(void);
void setupKerPropCanBus(void);
void setupBms(void);
void setupFansPumps(void);
void setupShaftRpm(void);
void setupTempSensors(void);
void setupThrottle(void);
// -----------------------------------------------------------------------------
// I2C defines
// -----------------------------------------------------------------------------
// #define SDA_PIN 27
// #define SCL_PIN 26
// -----------------------------------------------------------------------------
// 1-Wire data pin
// -----------------------------------------------------------------------------
const byte OneWirePin = 37;  // KerProp.1Wire
// -----------------------------------------------------------------------------
// Opto inputs for status of fans and pumps
// -----------------------------------------------------------------------------
const byte CoolantFanStatus = 39;   // KerProp.DigIn1
const byte CoolantPumpStatus = 40;  // KerProp.DigIn2
const byte AmbientFanStatus = 41;   // KerProp.DigIn3
const byte RpmProxyInputPin = 42;   // KerProp.RpmProxy
const byte SpareOptoInput5  = 45;   // KerProp.DigIn5
const byte SpareOptoInput6  = 46;   // KerProp.DigIn6
const byte SpareOptoInput7  = 47;   // KerProp.DigIn7
const byte SpareOptoInput8  = 48;   // KerProp.DigIn8
// -----------------------------------------------------------------------------
// Solid State Relay outputs to control relays for fans and pumps
// -----------------------------------------------------------------------------
const byte CoolantFanControl = 9;    // KerPRop.RelayControl1
const byte CoolantPumpControl = 10;  // KerProp.RelayControl2
const byte AmbientFanControl = 11;   // KerProp.RelayControl3
const byte SpareControl1 = 12;       // KerProp.RelayControl4
const byte SpareControl2 = 13;       // KerProp.RelayControl5
const byte SpareControl3 = 14;       // KerProp.RelayControl6
// -----------------------------------------------------------------------------
// MODBUS BMS defines --> using serial port 1
// -----------------------------------------------------------------------------
const byte ModBusDePin = 8;    // KerProp.MOD_DE
const byte ModBusTxdPin = 17;  // KerProp.MOD_TXD
const byte ModBusRxdPin = 18;  // KerProp.MOD_RXD
esp32ModbusRTU BMS_modbus(&Serial1, ModBusTxdPin, ModBusRxdPin, ModBusDePin);
sBMS_data_t sBMS0_data;
sBMS_data_t sBMS1_data;
// -----------------------------------------------------------------------------
// CAN defines --> using TWAI controller
// -----------------------------------------------------------------------------
const byte CAN_TX = 16;  // KerProp.CAN_TXD
const byte CAN_RX = 15;  // KerProp.CAN_RXD
const byte CAN_CLK = -1;
const byte CAN_BUS_OFF = -1;
ker_can myKerCan(239, 240);
// -----------------------------------------------------------------------------
//  Analog (Throttle)
// -----------------------------------------------------------------------------
const uint8_t kAnalogInputPin = 1;  // KerProp.ADC_IN1
// Define how often (in milliseconds) new samples are acquired
const unsigned int kAnalogInputReadInterval = 500;
// Define the produced value at the maximum input voltage (3.3V).
// A value of 3.3 gives output of 100%
// TODO I will make use of a lookup table as well as interpolating in the table
const float kAnalogInputScale = 100;
// -----------------------------------------------------------------------------
//  LCD
// -----------------------------------------------------------------------------
const byte LCD_CS = 7;     // KerProp.LCD_CS
const byte LCD_DC = 6;     // KerProp.LCD_DC
const byte LCD_MOSI = 35;  // KerProp.LCD_MOSI
const byte LCD_CLK = 0;    // KerProp.LCD_CLK
const byte LCD_RST = 5;    // KerProp.LCD_RST
const byte LCD_MISO = 36;  // KerProp.LCD_MISO
const byte LCD_LED = 4;    // KerProp.LCD_LED
SPIClass mySPI(FSPI);
ILI9488 tft = ILI9488(LCD_CS, LCD_DC, LCD_RST);
sDisplayData_t sDisplayData;
// -----------------------------------------------------------------------------
//  TEST DIGITAL OUTPUT (FREQ FOR RPM PULSE INPUT)
// -----------------------------------------------------------------------------
const byte TEST_DIG_OUTPUT = 3;
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
// TODO remove?? sDisplayData_t sDisplayData = {0};
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
  // -------------------------------------------------------------
  // KEROSHEBA PROPULSION ITEMS
//  setupThrottle();
  setupLcdDisplay();
  setupKerPropCanBus();
 // setupBms();
  setupFansPumps();
 //setupShaftRpm();
//  setupTempSensors();
  // Start networking, SK server connections and other SensESP internals
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

void setupLcdDisplay(void) {
  // -------------------------------------------------------------
  // LCD display
  // -------------------------------------------------------------
  pinMode(LCD_LED, OUTPUT);
  pinMode(LCD_RST, OUTPUT);
  digitalWrite(LCD_LED, 1);
  digitalWrite(LCD_RST, 0);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9488_BLACK);

  createFixedElements(&tft);
  // test data
  sDisplayData.Port.MotorTemp = 24;
  sDisplayData.Port.ControllerTemp = 25;
  sDisplayData.Port.MotorRpm = 1421;
  sDisplayData.Port.PhaseCurrent = 234;
  sDisplayData.Port.ControllerCommsOk = true;
  sDisplayData.Starboard.MotorTemp = 30;
  sDisplayData.Starboard.ControllerTemp = 35;
  sDisplayData.Starboard.MotorRpm = 1521;
  sDisplayData.Starboard.PhaseCurrent = 455;
  sDisplayData.Starboard.ControllerCommsOk = false;
  sDisplayData.ForwardBattery.Voltage = 54;
  sDisplayData.ForwardBattery.Current = 421;
  sDisplayData.ForwardBattery.SoC = 99;
  sDisplayData.ForwardBattery.Temp = 21;
  sDisplayData.ForwardBattery.BmsCommsOk = false;
  sDisplayData.AftBattery.Voltage = 56;
  sDisplayData.AftBattery.Current = 221;
  sDisplayData.AftBattery.SoC = 100;
  sDisplayData.AftBattery.Temp = 70;
  sDisplayData.AftBattery.BmsCommsOk = true;
  sDisplayData.CoolantFan = true;
  sDisplayData.AmbientFan = false;
  sDisplayData.CoolantPump = true;
  sDisplayData.InletTemp = 34;
  sDisplayData.OutletTemp = 23;
  sDisplayData.UpTime = 0;
  snprintf(sDisplayData.Ssid, strlen("Kerosheba") + 1, "Kerosheba");
  snprintf(sDisplayData.IpAddr, strlen("192.168.213.234") + 1,
           "192.168.213.234");
  sDisplayData.RunTime = 4353432;
  sDisplayData.ShaftRpm = 1232;
  createDynamicElements(&tft, sDisplayData);
  app.onRepeat(1000, []() {
    sDisplayData.UpTime++;
    createDynamicElements(&tft, sDisplayData);
  });
  // TODO testLcd();
}
void setupKerPropCanBus(void) {
  // -------------------------------------------------------------
  // CAN BUS
  // -------------------------------------------------------------
  Serial.println("Starting CAN BUS");
  myKerCan.begin(CAN_TX, CAN_RX, CAN_CLK, CAN_BUS_OFF);
  // Check the CAN bus receiver every 1ms
  app.onRepeat(1, []() { myKerCan.checkReceiver(); });
  // Send the CAN control command when in a state later than synced
  app.onRepeat(50, []() { 
    myKerCan.SendCommands(120, 100, 3);
    Serial.println("CAN Command sent");
     });
}
void setupBms(void) {
  // -------------------------------------------------------------
  // MOD BUS for 2x BMS
  // -------------------------------------------------------------
  BMS_modbus.onData([](uint8_t serverAddress, esp32Modbus::FunctionCode fc,
                       uint16_t address, uint8_t* data, size_t length) {
    // Serial.printf("Addr = 0x%04X\n", address);
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
      case 0:
        BMS_modbus.readHoldingRegisters(0x01, 0xD000, 0x0026);
        state_index++;
        break;
      case 1:
        BMS_modbus.readHoldingRegisters(0x01, 0xD026, 0x0019);
        state_index++;
        break;
      case 2:
        BMS_modbus.readHoldingRegisters(0x01, 0xD100, 0x0015);
        state_index++;
        break;
      case 3:
        BMS_modbus.readHoldingRegisters(0x01, 0xD200, 0x0001);
        state_index = 0;
        break;
    }
  });
}
void setupFansPumps(void) {
  // -------------------------------------------------------------
  // COOLANT FAN, PUMP AND AMBIENT FAN (ENGINE ROOM FAN)
  // -------------------------------------------------------------
  const char* sk_path_coolant_fan = "propulsion.switches.coolant.fan.state";
  const char* sk_path_coolant_pump = "propulsion.switches.coolant.pump.state";
  const char* sk_path_ambient_fan = "propulsion.switches.ambient.fan.state";
  const char* sk_path_spare1 = "propulsion.switches.spare1.state";
  const char* sk_path_spare2 = "propulsion.switches.spare2.state";
  const char* sk_path_spare3 = "propulsion.switches.spare3.state";
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
  const char* config_path_sk_output_spare1 = "/signalk/path/spare1";
  const char* config_path_sk_output_spare2 = "/signalk/path/spare2";
  const char* config_path_sk_output_spare3 = "/signalk/path/spare3";
  const char* config_path_repeat = "/signalk/repeat";

  // Create a digital output that is assumed to be connected to the
  // control channel of a relay or a MOSFET that will control the
  // coolant fan.
  pinMode(CoolantFanControl, OUTPUT);
  pinMode(CoolantPumpControl, OUTPUT);
  pinMode(AmbientFanControl, OUTPUT);
  pinMode(SpareControl1, OUTPUT);
  pinMode(SpareControl2, OUTPUT);
  pinMode(SpareControl3, OUTPUT);
  auto* coolant_fan_switch = new DigitalOutput(CoolantFanControl);
  auto* coolant_pump_switch = new DigitalOutput(CoolantPumpControl);
  auto* ambient_fan_switch = new DigitalOutput(AmbientFanControl);
  auto* spare1_switch = new DigitalOutput(SpareControl1);
  auto* spare2_switch = new DigitalOutput(SpareControl2);
  auto* spare3_switch = new DigitalOutput(SpareControl3);
  // Create a switch controller to handle the user press logic and
  // connect it to the load switch...
  SmartSwitchController* coolant_fan_controller = new SmartSwitchController();
  SmartSwitchController* coolant_pump_controller = new SmartSwitchController();
  SmartSwitchController* ambient_fan_controller = new SmartSwitchController();
  SmartSwitchController* spare1_controller = new SmartSwitchController();
  SmartSwitchController* spare2_controller = new SmartSwitchController();
  SmartSwitchController* spare3_controller = new SmartSwitchController();
  coolant_fan_controller->connect_to(coolant_fan_switch);
  coolant_pump_controller->connect_to(coolant_pump_switch);
  ambient_fan_controller->connect_to(ambient_fan_switch);
  spare1_controller->connect_to(spare1_switch);
  spare2_controller->connect_to(spare2_switch);
  spare3_controller->connect_to(spare3_switch);
  // Connect a physical button that will feed manual click types into the
  // controller.
  DigitalInputState* coolant_fan_status =
      new DigitalInputState(CoolantFanStatus, INPUT_PULLDOWN, 100);
  DigitalInputState* coolant_pump_status =
      new DigitalInputState(CoolantPumpStatus, INPUT_PULLDOWN, 100);
  DigitalInputState* ambient_fan_status =
      new DigitalInputState(AmbientFanStatus, INPUT_PULLDOWN, 100);
  DigitalInputState* spare5_status =
      new DigitalInputState(SpareOptoInput5, INPUT_PULLDOWN, 100);
  DigitalInputState* spare6_status =
      new DigitalInputState(SpareOptoInput6, INPUT_PULLDOWN, 100);
  DigitalInputState* spare7_status =
      new DigitalInputState(SpareOptoInput7, INPUT_PULLDOWN, 100);
  DigitalInputState* spare8_status =
      new DigitalInputState(SpareOptoInput8, INPUT_PULLDOWN, 100);
  PressRepeater* coolant_fan_pr = new PressRepeater();
  PressRepeater* coolant_pump_pr = new PressRepeater();
  PressRepeater* ambient_fan_pr = new PressRepeater();
  PressRepeater* spare5_pr = new PressRepeater();
  PressRepeater* spare6_pr = new PressRepeater();
  PressRepeater* spare7_pr = new PressRepeater();
  PressRepeater* spare8_pr = new PressRepeater();
  coolant_fan_status->connect_to(coolant_fan_pr);
  coolant_pump_status->connect_to(coolant_pump_pr);
  ambient_fan_status->connect_to(ambient_fan_pr);
  spare5_status->connect_to(spare5_pr);
  spare6_status->connect_to(spare6_pr);
  spare7_status->connect_to(spare7_pr);
  spare8_status->connect_to(spare8_pr);
  coolant_fan_pr->connect_to(new ClickType(config_path_button_c))
      ->connect_to(coolant_fan_controller);
  coolant_pump_pr->connect_to(new ClickType(config_path_button_c))
      ->connect_to(coolant_pump_controller);
  ambient_fan_pr->connect_to(new ClickType(config_path_button_c))
      ->connect_to(ambient_fan_controller);
  spare5_pr->connect_to(new ClickType(config_path_button_c))
      ->connect_to(spare1_controller);
  spare6_pr->connect_to(new ClickType(config_path_button_c))
      ->connect_to(spare2_controller);
  spare7_pr->connect_to(new ClickType(config_path_button_c))
      ->connect_to(spare3_controller);
  spare8_pr->connect_to(new ClickType(config_path_button_c))
      ->connect_to(spare3_controller); //NOTE that this is the same controller is for spare7_pr
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
  
  auto* sk_listener_spare1=
      new StringSKPutRequestListener(sk_path_spare1);
  sk_listener_spare1->connect_to(spare1_controller);
  
  auto* sk_listener_spare2=
      new StringSKPutRequestListener(sk_path_spare2);
  sk_listener_spare2->connect_to(spare2_controller);
  
  auto* sk_listener_spare3=
      new StringSKPutRequestListener(sk_path_spare3);
  sk_listener_spare3->connect_to(spare3_controller);
  // Finally, connect the load switch to an SKOutput so it reports its state
  // to the Signal K server.  Since the load switch only reports its state
  // whenever it changes (and switches like light switches change
  // infrequently),
  // send it through a `RepeatReport` transform, which will cause the state
  // to be reported to the server every 10 seconds, regardless of whether
  // or not it has changed.  That keeps the value on the server fresh and
  // lets the server know the switch is still alive.
  coolant_fan_switch
      ->connect_to(new RepeatReport<bool>(1000, config_path_repeat))
      ->connect_to(new SKOutputBool(sk_path_coolant_fan,
                                    config_path_sk_output_coolant_fan));

  coolant_pump_switch
      ->connect_to(new RepeatReport<bool>(1000, config_path_repeat))
      ->connect_to(new SKOutputBool(sk_path_coolant_pump,
                                    config_path_sk_output_coolant_pmp));

  ambient_fan_switch
      ->connect_to(new RepeatReport<bool>(1000, config_path_repeat))
      ->connect_to(new SKOutputBool(sk_path_ambient_fan,
                                    config_path_sk_output_ambient_fan));

  spare1_switch
      ->connect_to(new RepeatReport<bool>(1000, config_path_repeat))
      ->connect_to(new SKOutputBool(sk_path_spare1,
                                    config_path_sk_output_spare1));

  spare2_switch
      ->connect_to(new RepeatReport<bool>(1000, config_path_repeat))
      ->connect_to(new SKOutputBool(sk_path_spare2,
                                    config_path_sk_output_spare2));

  spare3_switch
      ->connect_to(new RepeatReport<bool>(1000, config_path_repeat))
      ->connect_to(new SKOutputBool(sk_path_spare3,
                                    config_path_sk_output_spare3));
}
void setupShaftRpm(void) {
  // -------------------------------------------------------------
  // SHAFT RPM
  // -------------------------------------------------------------
  // Set GPIO pin to output and toggle it every kDigitalOutputInterval
  // const uint8_t kDigitalOutputPin = TEST_DIG_OUTPUT;
  // const unsigned int kDigitalOutputInterval = 20;
  // pinMode(kDigitalOutputPin, OUTPUT);
  // app.onRepeat(kDigitalOutputInterval, [kDigitalOutputPin]() {
  //   digitalWrite(kDigitalOutputPin, !digitalRead(kDigitalOutputPin));
  // });
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
  pinMode(RpmProxyInputPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(RpmProxyInputPin), isr, RISING);
  auto* rpm_sensor = new RepeatSensor<float>(1000, []() { return (shaftHz); });
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
  app.onRepeat(1000, []() {
    if ((int)shaftHz != (int)clShaftFreq.getCurrentFrequency()) {
      shaftHz = clShaftFreq.getCurrentFrequency();
    }
  });
}
void setupTempSensors(void) {
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
      new OneWireTemperature(dts, 2000, "/2_oneWire/starboardControllerTemp");
  auto* temp_sensor_engineRoom =
      new OneWireTemperature(dts, 2000, "/2_oneWire/EngineRoomTemp");
  auto* temp_sensor_coolant =
      new OneWireTemperature(dts, 2000, "/2_oneWire/CoolantTemp");
  // Create the SK metadata for each sensor
  sDisplayScale_t displayScale;
  displayScale.lowerDisplayScale = -40;
  displayScale.upperDisplayScale = 120;
  displayScale.typeDisplayScale = "linear";
  // zones meta information for Temp (hardcoded to 4 but some can be left blank)
  std::array<sZone_t, 4> zonesTemp;
  zonesTemp.at(0).lowerZone = -40;
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
      displayScale, {"visual", "sound"},  // alert method (visual or sound)
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
      displayScale, {"visual", "sound"},  // alert method (visual or sound)
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
      displayScale, {"visual", "sound"},  // alert method (visual or sound)
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
      displayScale, {"visual", "sound"},   // alert method (visual or sound)
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
      displayScale, {"visual", "sound"},  // alert method (visual or sound)
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
      displayScale, {"visual", "sound"},  // alert method (visual or sound)
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
}
void setupThrottle(void) {
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
}
