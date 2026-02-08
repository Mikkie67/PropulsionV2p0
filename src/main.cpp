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
#include "SPIFFS.h"
#include <memory>

#include "sensesp.h"
#include "bms_parser.h"
#include "esp32ModbusRTU.h"
#include "ker_can.hpp"
#include "propDisplay.h"
#include "rpm_class.hpp"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include <sensesp/transforms/hysteresis.h>

using namespace sensesp;
using namespace sensesp::onewire;

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
const uint16_t TEMP_SENSOR_READ_INTERVAL = 1000;  // Temperature sensor read interval in milliseconds
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
// if i add all the signals, including the clk,miso,mosi in the init function, 
// it seems to not use the hwSPI
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

// Fan/pump state tracking
bool coolant_fan_state = false;      // Current state of coolant fan relay
bool coolant_pump_state = false;     // Current state of coolant pump relay
bool ambient_fan_state = false;      // Current state of ambient fan relay
bool spare1_relay_state = false;      // Current state of spare1 relay
bool spare2_relay_state = false;      // Current state of spare2 relay
bool spare3_relay_state = false;      // Current state of spare3 relay

// Motor runtime tracking (persistent across sessions)
bool motors_running = false;         // Current motor running state
unsigned long motor_runtime_seconds = 0;  // Total accumulated motor runtime

// Coolant pump max temperature tracking - updated by sensor callbacks
float max_motor_controller_temp_kelvin = 273.15f;  // Track max of 4 temps in Kelvin for pump control


// Forward declarations - initialized in setupTempSensors() after sensesp_app is created
DallasTemperatureSensors* dts = nullptr;
OneWireTemperature* temp_sensor_portMotor = nullptr;
OneWireTemperature* temp_sensor_starboardMotor = nullptr;
OneWireTemperature* temp_sensor_portController = nullptr;
OneWireTemperature* temp_sensor_starboardController = nullptr;
OneWireTemperature* temp_sensor_engineRoom = nullptr;
OneWireTemperature* temp_sensor_coolant = nullptr;

// Hysteresis transforms for fan/pump control - initialized in setupFansPumps() after sensesp_app is created
std::shared_ptr<sensesp::Hysteresis<float, bool>> coolant_fan_hyst = nullptr;
std::shared_ptr<sensesp::Hysteresis<float, bool>> coolant_pump_hyst = nullptr;
std::shared_ptr<sensesp::Hysteresis<float, bool>> ambient_fan_hyst = nullptr;



// =========================================================
// FAN/PUMP Control functions
// =========================================================
auto coolant_fan_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>(
  [](bool on) {
    // drive your GPIO / relay here
    digitalWrite(CoolantFanControl, on ? HIGH : LOW);
    if (on != coolant_fan_state) {
      coolant_fan_state = on;
      sDisplayData.CoolantFan = on;
      debugD("Coolant fan command: %s", on ? "ON" : "OFF");
    }
  }
);
auto ambient_fan_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>(
  [](bool on) {
    // drive your GPIO / relay here
    digitalWrite(AmbientFanControl, on ? HIGH : LOW);
    if (on != ambient_fan_state) {
      ambient_fan_state = on;
      sDisplayData.AmbientFan = on;
      debugD("Ambient fan command: %s", on ? "ON" : "OFF");
    }
  }
);
auto coolant_pump_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>(
  [](bool on) {
    // drive your GPIO / relay here
    digitalWrite(CoolantPumpControl, on ? HIGH : LOW);
    if (on != coolant_pump_state) {
      coolant_pump_state = on;
      sDisplayData.CoolantPump = on;
     debugD("Coolant pump command: %s", on ? "ON" : "OFF");
    }
  }
);
auto spare1_relay_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>(
  [](bool on) {
    // drive your GPIO / relay here
    digitalWrite(SpareControl1, on ? HIGH : LOW);
    if (on != spare1_relay_state) {
      spare1_relay_state = on;
      debugD("Spare1 relay command: %s", on ? "ON" : "OFF");
    }
  }
);  
auto spare2_relay_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>(
  [](bool on) {
    // drive your GPIO / relay here
    digitalWrite(SpareControl2, on ? HIGH : LOW);
    if (on != spare2_relay_state) {
      spare2_relay_state = on;
      debugD("Spare2 relay command: %s", on ? "ON" : "OFF");
    }
  }
);
auto spare3_relay_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>(
  [](bool on) {
    // drive your GPIO / relay here
    digitalWrite(SpareControl3, on ? HIGH : LOW);
    if (on != spare3_relay_state) {
      spare3_relay_state = on;
      debugD("Spare3 relay command: %s", on ? "ON" : "OFF");
    }
  }
);

 
// =========================================================
// THROTTLE CONFIGURATION CLASS - Persistent storage via SPIFFS
// =========================================================
class ThrottleConfig : public FileSystemSaveable {
 private:
  float max_phase_current_ = 250.0f;
  unsigned long can_command_interval_ = 500;
  
 public:
  ThrottleConfig(String config_path = "")
      : FileSystemSaveable(config_path) {
    this->load();
  }
  
  float get_max_phase_current() const { return max_phase_current_; }
  void set_max_phase_current(float current) {
    max_phase_current_ = current;
    this->save();
  }
  
  unsigned long get_can_command_interval() const { return can_command_interval_; }
  void set_can_command_interval(unsigned long interval) {
    can_command_interval_ = interval;
    this->save();
  }
  
  virtual bool to_json(JsonObject& root) override {
    root["max_phase_current"] = max_phase_current_;
    root["can_command_interval"] = can_command_interval_;
    return true;
  }
  
  virtual bool from_json(const JsonObject& config) override {
    if (!config["max_phase_current"].is<float>()) {
      return false;
    }
    max_phase_current_ = config["max_phase_current"].as<float>();
    if (config["can_command_interval"].is<unsigned long>()) {
      can_command_interval_ = config["can_command_interval"].as<unsigned long>();
    }
    return true;
  }
};

// Global throttle configuration instance
ThrottleConfig* throttle_config = nullptr;
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
  SetupLogging(ESP_LOG_DEBUG);
  File root, file;  // Declare SPIFFS file variables

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Propulsion5")
                    // Enable WiFi Access Point mode (SensESP 3.x API)
                    //->set_wifi_access_point("Propulsion5", "")  // Empty password for open network
// Optionally, hard-code the WiFi and Signal K server
// settings. This is normally not needed.
//->set_wifi("My WiFi SSID", "my_wifi_password")
//->set_sk_server("192.168.10.3", 80)
#ifdef OTA_ENABLED
                    ->enable_ota("Regurk67!")
#endif
                    ->get_app();
  debugD("SensESP App created");
  
  // Check SPIFFS files AFTER SensESP initializes filesystem
  delay(500);
  debugD("FILES IN SPIFFS ROOT (after SensESP init):");
  root = SPIFFS.open("/");
  if (!root) {
    debugE("ERROR: Cannot open root directory!");
  } else {
    file = root.openNextFile();
    bool has_files = false;
    while (file) {
      debugD("  FILE: %s (size: %d bytes)", file.name(), file.size());
      has_files = true;
      file = root.openNextFile();
    }
    if (!has_files) {
      debugW("  (SPIFFS is EMPTY - no files found!)");
    }
  }
  
  // -------------------------------------------------------------
  // KEROSHEBA PROPULSION ITEMS
  debugD("Starting setupThrottle()");
  setupThrottle();
  debugD("setupThrottle() complete");
  
  debugD("Starting setupLcdDisplay()");
  setupLcdDisplay();
  debugD("setupLcdDisplay() complete");
  
  debugD("Starting setupKerPropCanBus()");
  setupKerPropCanBus();
  debugD("setupKerPropCanBus() complete");
  
 // setupBms();

  debugD("Starting setupTempSensors()");
  setupTempSensors();
  debugD("setupTempSensors() complete");
  
  debugD("Starting setupFansPumps()");
  setupFansPumps();
  debugD("setupFansPumps() complete");
  
  debugD("Starting setupShaftRpm()");
  setupShaftRpm();
  debugD("setupShaftRpm() complete");
  
  
  // Start networking, SK server connections and other SensESP internals
  debugD("Starting sensesp_app->start()");
  debugD("About to call start() - SensESP HTTP server and WiFi AP should initialize here");
  delay(100);
  sensesp_app->start();
  debugD("sensesp_app->start() complete");
  
  /*// Force WiFi AP mode if not already in AP mode
  if ((WiFi.getMode() & WIFI_AP) == 0) {
    debugW("WiFi AP not active! Forcing AP mode...");
    WiFi.mode(WIFI_AP);
    delay(100);
    WiFi.softAP("Propulsion5", "");  // Empty password for open network
    delay(500);
  }*/
  
  // Check WiFi status
  debugI("WiFi Status:");
  debugI("  WiFi Mode: %d (1=STA, 2=AP, 3=STA+AP)", WiFi.getMode());
  debugI("  AP SSID: %s", WiFi.softAPSSID().c_str());
  debugI("  AP IP: %s", WiFi.softAPIP().toString().c_str());
  debugI("  AP Active: %s", (WiFi.getMode() & WIFI_AP) ? "YES" : "NO");
  
  debugD("Web server should be running on http://192.168.4.1");
  
  // Add some delay for SensESP HTTP server to fully initialize
  delay(2000);
  debugI("Waiting complete. HTTP server should now be fully initialized.");
}
void loop() {
  app.tick();
  event_loop()->tick();
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
  // Initialize display data with sentinel values (--) until real values arrive
  sDisplayData.Port.MotorTemp = -1;
  sDisplayData.Port.ControllerTemp = -1;
  sDisplayData.Port.MotorRpm = -1;
  sDisplayData.Port.PhaseCurrent = -1;
  sDisplayData.Port.ControllerCommsOk = false;
  sDisplayData.Starboard.MotorTemp = -1;
  sDisplayData.Starboard.ControllerTemp = -1;
  sDisplayData.Starboard.MotorRpm = -1;
  sDisplayData.Starboard.PhaseCurrent = -1;
  sDisplayData.Starboard.ControllerCommsOk = false;
  sDisplayData.ForwardBattery.Voltage = -1;
  sDisplayData.ForwardBattery.Current = -1;
  sDisplayData.ForwardBattery.SoC = -1;
  sDisplayData.ForwardBattery.Temp = -1;
  sDisplayData.ForwardBattery.BmsCommsOk = false;
  sDisplayData.AftBattery.Voltage = -1;
  sDisplayData.AftBattery.Current = -1;
  sDisplayData.AftBattery.SoC = -1;
  sDisplayData.AftBattery.Temp = -1;
  sDisplayData.AftBattery.BmsCommsOk = false;
  sDisplayData.CoolantFan = false;
  sDisplayData.AmbientFan = false;
  sDisplayData.CoolantPump = false;
  sDisplayData.AmbientTemp = -1;
  sDisplayData.CoolantTemp = -1;
  sDisplayData.UpTime = 0;
  // Get actual WiFi network SSID and station IP address
  String ssid_str = WiFi.SSID();
  snprintf(sDisplayData.Ssid, sizeof(sDisplayData.Ssid), "%s", ssid_str.c_str());
  String ip_str = WiFi.localIP().toString();
  snprintf(sDisplayData.IpAddr, sizeof(sDisplayData.IpAddr), "%s", ip_str.c_str());
  sDisplayData.RunTime = 0;
  sDisplayData.ShaftRpm = -1;
  createDynamicElements(&tft, sDisplayData);
  app.onRepeat(1000, [&]() {
    sDisplayData.UpTime++;
    debugI("Display update: AmbientTemp=%d, CoolantTemp=%d", sDisplayData.AmbientTemp, sDisplayData.CoolantTemp);
    createDynamicElements(&tft, sDisplayData);
  });
  // TODO testLcd();
}
void setupKerPropCanBus(void) {
  // TEMPORARILY DISABLED FOR DEBUGGING GPIO 238 ERROR
  /*
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
  */
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
  // =========================================================
  // COOLANT FAN, PUMP AND AMBIENT FAN - Temperature controlled
  // =========================================================
  
  // Create digital outputs for relay control
  pinMode(CoolantFanControl, OUTPUT);
  pinMode(CoolantPumpControl, OUTPUT);
  pinMode(AmbientFanControl, OUTPUT);
  pinMode(SpareControl1, OUTPUT);
  pinMode(SpareControl2, OUTPUT);
  pinMode(SpareControl3, OUTPUT);
  
  // Create Signal K outputs for fan/pump status monitoring
  // auto* coolant_fan_output = new SKOutputBool(
  //     "propulsion.cooling.coolantFan",
  //     new SKMetadata("", "Coolant Fan", "Coolant fan relay status")
  // );
  
  // auto* coolant_pump_output = new SKOutputBool(
  //     "propulsion.cooling.coolantPump",
  //     new SKMetadata("", "Coolant Pump", "Coolant pump relay status")
  // );
  
  // auto* ambient_fan_output = new SKOutputBool(
  //     "propulsion.cooling.ambientFan",
  //     new SKMetadata("", "Ambient Fan", "Engine room ambient fan relay status")
  // );
  
  // =========================================================
  // COOLANT FAN - Controlled by coolant temperature
  // =========================================================
  
  // Initialize hysteresis transform for coolant fan control
  coolant_fan_hyst = std::make_shared<sensesp::Hysteresis<float, bool>>(
       303.15f,   // lower threshold (OFF) in Kelvin (30°C)
       305.15f,   // upper threshold (ON) in Kelvin (32°C)
       false,     // low_output
       true,      // high_output
       "/coolant_fan_control"
  );

  temp_sensor_coolant->connect_to(coolant_fan_hyst)->connect_to(coolant_fan_cmd);
  ConfigItem(coolant_fan_hyst.get())
    ->set_title("Coolant Fan Control")
    ->set_description("Hysteresis control with ON/OFF thresholds in Kelvin (add 273.15 to convert from Celsius).")
    ->set_sort_order(200);
   
  // =========================================================
  // COOLANT PUMP - ON if ANY motor/controller exceeds threshold, OFF when ALL below
  // =========================================================
  
  // Initialize hysteresis transform for coolant pump control using max motor/controller temp
  coolant_pump_hyst = std::make_shared<sensesp::Hysteresis<float, bool>>(
       313.15f,   // lower threshold (OFF) in Kelvin (40°C)
       315.15f,   // upper threshold (ON) in Kelvin (42°C)
       false,     // low_output
       true,      // high_output
       "/coolant_pump_control"
  );

  coolant_pump_hyst->connect_to(coolant_pump_cmd);
  ConfigItem(coolant_pump_hyst.get())
    ->set_title("Coolant Pump Control")
    ->set_description("Hysteresis control based on max motor/controller temperature. ON/OFF thresholds in Kelvin (add 273.15 to convert from Celsius).")
    ->set_sort_order(201);
  
  // Periodic feedback of max motor/controller temp to hysteresis
  app.onRepeat(1000, [&]() {
    // Calculate the max of the 4 current temperatures (in Celsius, convert to Kelvin)
    float temps_celsius[] = {portMotor_temperature, starboardMotor_temperature, 
                             portController_temperature, starboardController_temperature};
    float max_celsius = temps_celsius[0];
    for (int i = 1; i < 4; i++) {
      if (temps_celsius[i] > max_celsius) {
        max_celsius = temps_celsius[i];
      }
    }
    max_motor_controller_temp_kelvin = max_celsius + 273.15f;
    coolant_pump_hyst->set(max_motor_controller_temp_kelvin);
  });
  
  // =========================================================
  // AMBIENT FAN - Controlled by engine room temperature
  // =========================================================
  
  // Initialize hysteresis transform for ambient fan control
  ambient_fan_hyst = std::make_shared<sensesp::Hysteresis<float, bool>>(
       308.15f,   // lower threshold (OFF) in Kelvin (35°C)
       310.15f,   // upper threshold (ON) in Kelvin (37°C)
       false,     // low_output
       true,      // high_output
       "/ambient_fan_control"
  );
  
  temp_sensor_engineRoom->connect_to(ambient_fan_hyst)->connect_to(ambient_fan_cmd);
  ConfigItem(ambient_fan_hyst.get())
    ->set_title("Ambient Fan Control")
    ->set_description("Hysteresis control for engine room ambient temperature cooling.")
    ->set_sort_order(202);
} 
void setupShaftRpm(void) {
  // Shaft RPM monitoring - reads pulses from propeller speed sensor via interrupt
  // Updates shaft frequency based on pulse intervals
  
  // Setup RPM input pin and interrupt handler
  pinMode(RpmProxyInputPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(RpmProxyInputPin), isr, RISING);
  
  // Create Signal K output for shaft RPM with metadata
  auto* shaft_rpm_output = new SKOutputNumeric<float>(
      "propulsion.0.speed", 
      "/shaft_rpm",  // config path for WebUI
      new SKMetadata(
          "rpm",  // units
          "Shaft RPM",  // display_name
          "Propeller shaft rotational speed in revolutions per minute"  // description
      )
  );
  
  // Zone configuration is handled through the WebUI interface
  // Recommended zones in WebUI:
  //   0-1000 RPM: Green (nominal)
  //   1000-1200 RPM: Yellow (warn)
  //   1200-1500 RPM: Orange (alarm)
  //   1500+ RPM: Red (fault)
  
  // Periodic update of shaft RPM to Signal K and motor runtime tracking
  app.onRepeat(1000, [shaft_rpm_output]() {
    double frequency = clShaftFreq.getCurrentFrequency();
    float rpm = frequency * 60;
    shaft_rpm_output->set(rpm);
    
    // Track motor runtime: start when RPM > 100, stop when RPM < 20
    if (rpm > 100.0f && !motors_running) {
      motors_running = true;
      debugI("Motors started - beginning runtime accumulation");
    } else if (rpm < 20.0f && motors_running) {
      motors_running = false;
      debugI("Motors stopped - paused runtime accumulation at %lu seconds", motor_runtime_seconds);
    }
    
    // Increment runtime when motors are running
    if (motors_running) {
      motor_runtime_seconds++;
      sDisplayData.RunTime = (uint32_t)(motor_runtime_seconds / 60);  // Convert to minutes for display
    }
    
    if (frequency > 0) {
      debugI("Shaft RPM: %.0f (Zone determination in UI)", rpm);
    }
  });
}
void setupTempSensors(void) {
  // Temperature sensors setup - 6 sensors with configuration data in WebUI
  // OneWire temperature sensors on pin 37
  
  // Give the OneWire sensor(s) time to stabilize and be ready before bus scan
  // Sensor power-up and response time is critical for proper discovery
  debugI("Initializing OneWire bus on pin %d, waiting for sensor stabilization...", OneWirePin);
  delay(1000);
  
  // Initialize OneWire sensors AFTER sensesp_app is created in setup()
  dts = new DallasTemperatureSensors(OneWirePin);
  temp_sensor_portMotor = new OneWireTemperature(dts, TEMP_SENSOR_READ_INTERVAL, "/2_oneWire/portMotorTemp");
  temp_sensor_starboardMotor = new OneWireTemperature(dts, TEMP_SENSOR_READ_INTERVAL, "/2_oneWire/starboardMotorTemp");
  temp_sensor_portController = new OneWireTemperature(dts, TEMP_SENSOR_READ_INTERVAL, "/2_oneWire/portControllerTemp");
  temp_sensor_starboardController = new OneWireTemperature(dts, TEMP_SENSOR_READ_INTERVAL, "/2_oneWire/starboardControllerTemp");
  temp_sensor_engineRoom = new OneWireTemperature(dts, TEMP_SENSOR_READ_INTERVAL, "/2_oneWire/EngineRoomTemp");
  temp_sensor_coolant = new OneWireTemperature(dts, TEMP_SENSOR_READ_INTERVAL, "/2_oneWire/CoolantTemp");
    
  debugI("OneWire sensors detected on pin %d", OneWirePin);
  
  // Config Items for the UI
  ConfigItem(temp_sensor_portMotor)
    ->set_title("Port Motor Temperature Sensor")
    ->set_description("Temperature sensor for the port motor.")
    ->set_sort_order(100);
  ConfigItem(temp_sensor_starboardMotor)
    ->set_title("Starboard Motor Temperature Sensor")
    ->set_description("Temperature sensor for the starboard motor.")
    ->set_sort_order(101);
  ConfigItem(temp_sensor_portController)
    ->set_title("Port Controller Temperature Sensor")
    ->set_description("Temperature sensor for the port motor controller.")
    ->set_sort_order(102);
  ConfigItem(temp_sensor_starboardController)
    ->set_title("Starboard Controller Temperature Sensor")
    ->set_description("Temperature sensor for the starboard motor controller.")
    ->set_sort_order(103);
  ConfigItem(temp_sensor_engineRoom)
    ->set_title("Engine Room Temperature Sensor")
    ->set_description("Temperature sensor for the engine room ambient temperature.")
    ->set_sort_order(104);
  ConfigItem(temp_sensor_coolant)
    ->set_title("Coolant Temperature Sensor")
    ->set_description("Temperature sensor for the motor coolant temperature.")
    ->set_sort_order(105);


  // Connect temperature sensors to global variables and Signal K outputs
  // Create Signal K outputs
  auto* port_motor_sk = new SKOutputFloat("propulsion.port.temperature");
  auto* starboard_motor_sk = new SKOutputFloat("propulsion.starboard.temperature");
  auto* port_ctrl_sk = new SKOutputFloat("electrical.controllers.port.temperature");
  auto* starboard_ctrl_sk = new SKOutputFloat("electrical.controllers.starboard.temperature");
  auto* engine_room_sk = new SKOutputFloat("propulsion.engineRoom.temperature");
  auto* coolant_sk = new SKOutputFloat("propulsion.coolant.temperature");
  
  // Port Motor Temperature
  temp_sensor_portMotor->connect_to(
      new LambdaConsumer<float>([port_motor_sk](const float& temp) {
        float celsius = temp - 273.15f;
        portMotor_temperature = celsius;
        sDisplayData.Port.MotorTemp = (int)celsius;
        port_motor_sk->set(temp);
      })
  );
  
  // Starboard Motor Temperature
  temp_sensor_starboardMotor->connect_to(
      new LambdaConsumer<float>([starboard_motor_sk](const float& temp) {
        float celsius = temp - 273.15f;
        starboardMotor_temperature = celsius;
        sDisplayData.Starboard.MotorTemp = (int)celsius;
        starboard_motor_sk->set(temp);
      })
  );
  
  // Port Controller Temperature
  temp_sensor_portController->connect_to(
      new LambdaConsumer<float>([port_ctrl_sk](const float& temp) {
        float celsius = temp - 273.15f;
        portController_temperature = celsius;
        sDisplayData.Port.ControllerTemp = (int)celsius;
        port_ctrl_sk->set(temp);
      })
  );
  
  // Starboard Controller Temperature
  temp_sensor_starboardController->connect_to(
      new LambdaConsumer<float>([starboard_ctrl_sk](const float& temp) {
        float celsius = temp - 273.15f;
        starboardController_temperature = celsius;
        sDisplayData.Starboard.ControllerTemp = (int)celsius;
        starboard_ctrl_sk->set(temp);
      })
  );
  
  // Engine Room Temperature
  temp_sensor_engineRoom->connect_to(
      new LambdaConsumer<float>([engine_room_sk](const float& temp) {
        float celsius = temp - 273.15f;
        engineroom_temperature = celsius;
        sDisplayData.AmbientTemp = (int)celsius;
        engine_room_sk->set(temp);
      })
  );
  
  // Coolant Temperature
  temp_sensor_coolant->connect_to(
      new LambdaConsumer<float>([coolant_sk](const float& temp) {
        float celsius = temp - 273.15f;
        coolant_temperature = celsius;
        sDisplayData.CoolantTemp = (int)celsius;
        coolant_sk->set(temp);
      })
  );
}
void setupThrottle(void) {
  // Initialize persistent throttle configuration
  throttle_config = new ThrottleConfig("/throttle_config");
  
  // Configure ADC for analog throttle reading
  analogSetAttenuation(ADC_11db);  // Full range 0-3.3V
  
  // Create Signal K output for ADC throttle input
  auto* adc_throttle_output = new SKOutputNumeric<float>(
      "propulsion.0.throttlePosition",
      "/adc_throttle",
      new SKMetadata("ratio", "ADC Throttle", "Throttle lever position from ADC potentiometer (0-100%)")
  );
  
  // Read ADC throttle and send commands to both motors
  app.onRepeat(throttle_config->get_can_command_interval(), [adc_throttle_output]() {
    // Read ADC throttle in millivolts (0-3300mV)
    uint16_t raw_mv = analogReadMilliVolts(kAnalogInputPin);
    // Convert to percentage (0-100%)
    float throttle_percent = (raw_mv / 3300.0f) * 100.0f;
    // Clamp to 0-100%
    if (throttle_percent > 100.0f) throttle_percent = 100.0f;
    if (throttle_percent < 0.0f) throttle_percent = 0.0f;
    
    // Output to Signal K
    adc_throttle_output->set(throttle_percent);
    
    // Send throttle command to both motors (Port and Starboard)
    // Convert throttle percentage to target current using persistent config
    int16_t target_current = (int16_t)((throttle_percent / 100.0f) * throttle_config->get_max_phase_current());
    
    // Send to Port motor
    myKerCan.McuPort.SendCommand(target_current, 0, 0);
    // Send to Starboard motor
    myKerCan.McuStarboard.SendCommand(target_current, 0, 0);
    
    //debugD("ADC Throttle: %.1f%% -> Current: %d A", throttle_percent, target_current);
  });
  
  // =========================================================
  // THROTTLE FEEDBACK - From both motor controllers
  // =========================================================
  
  // Create Signal K outputs for both motor throttle feedback
  auto* port_throttle_fb = new SKOutputNumeric<float>(
      "propulsion.0.port.throttlePosition",
      "/port_throttle_fb",
      new SKMetadata("ratio", "Port Motor Throttle", "Throttle feedback from Port motor controller")
  );
  
  auto* starboard_throttle_fb = new SKOutputNumeric<float>(
      "propulsion.1.throttlePosition",
      "/starboard_throttle_fb",
      new SKMetadata("ratio", "Starboard Motor Throttle", "Throttle feedback from Starboard motor controller")
  );
  
  // Read motor throttle feedback every 1000ms
  app.onRepeat(1000, [port_throttle_fb, starboard_throttle_fb]() {
    // Port motor throttle feedback
    int16_t port_throttle_raw = myKerCan.McuPort.getThrottlePosition();
    float port_throttle_percent = (float)port_throttle_raw;
    if (port_throttle_percent > 100.0f) port_throttle_percent = 100.0f;
    if (port_throttle_percent < 0.0f) port_throttle_percent = 0.0f;
    port_throttle_fb->set(port_throttle_percent);
    
    // Starboard motor throttle feedback
    int16_t starboard_throttle_raw = myKerCan.McuStarboard.getThrottlePosition();
    float starboard_throttle_percent = (float)starboard_throttle_raw;
    if (starboard_throttle_percent > 100.0f) starboard_throttle_percent = 100.0f;
    if (starboard_throttle_percent < 0.0f) starboard_throttle_percent = 0.0f;
    starboard_throttle_fb->set(starboard_throttle_percent);
    
    //debugD("Port throttle: %.0f%% | Starboard throttle: %.0f%%", 
    //       port_throttle_percent, starboard_throttle_percent);
  });
}
