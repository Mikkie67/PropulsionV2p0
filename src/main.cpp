// Kerosheba Propulsion boat controller (VCU)

// TODO
//  get throttle value from ADC
//  add UI elements for CAN bus
//       control mode
//       mcu ids
//       vcu id

// #define SERIAL_DEBUG_DISABLED
#define OTA_ENABLED
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
#include "signalk_metadata_zones.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include <sensesp/transforms/hysteresis.h>
#include "sensesp/ui/ui_controls.h"
#include "sensesp/signalk/signalk_delta_queue.h"

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
uint16_t last_requested_address = 0;  // Track which address was just requested for CRC error debugging
uint8_t last_requested_slave = 0;     // Track which slave address was requested
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
const uint8_t kAnalogInputPin = 2;  // KerProp.ADC_IN2 (GPIO2)
// Define how often (in milliseconds) new samples are acquired
const unsigned int kAnalogInputReadInterval = 500;
// Define the produced value at the maximum input voltage (3.3V).
// A value of 3.3 gives output of 100%
// TODO I will make use of a lookup table as well as interpolating in the table
const float kAnalogInputScale = 100;
// ADC reference voltage in volts
float adc_reference_voltage = 5.0f;
// ADC maximum register value (12-bit)
const uint16_t ADC_MAX_VALUE = 4095;
// ADC midpoint value for bidirectional throttle (neutral position)
float adc_midpoint_voltage = 2.5f;
// Hysteresis band around neutral position (in volts) - throttle remains at 0% within this range
float adc_neutral_hysteresis = 0.1f;
// Throttle percentage threshold below which motor enters idle mode
float throttle_idle_threshold = 0.10f;  // % (absolute value)
// Throttle percentage to use when in idle mode
float throttle_idle_percent = 0.10f;  // %
// Maximum phase current for motor control (in Amps)
float max_phase_current = 250.0f;  // A

// NumberConfig objects for persistent storage and UI configuration
NumberConfig* config_adc_reference_voltage = nullptr;
NumberConfig* config_adc_neutral_hysteresis = nullptr;
NumberConfig* config_throttle_idle_threshold = nullptr;
NumberConfig* config_throttle_idle_percent = nullptr;
NumberConfig* config_max_phase_current = nullptr;
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

// Cell metadata tracking - sent when Signal K websocket connects
bool was_signalk_connected = false;
int metadata_batch_index = 0;  // Track which batch of metadata to send
std::vector<String> cell_metadata_messages;  // Pre-built metadata messages

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
// Global throttle configuration instance - removed, using constants
// =========================================================
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

// Function to build and store cell metadata messages (called during setup)
void build_cell_metadata() {
  // Battery 01 metadata
  String bat01_description = "Battery 01 cell voltage";
  for (int i = 0; i < 32; i++) {
    String meta_msg = "{\"path\":\"electrical.batteries.bat01.cells.cell" + String(i+1) + "\",\"value\":null,\"meta\":{\"units\":\"V\",\"description\":\"" + bat01_description + "\",\"displayName\":\"Cell " + String(i+1) + "\",\"warnLower\":2.9,\"faultLower\":2.6,\"warnUpper\":3.55,\"faultUpper\":3.65}}";
    cell_metadata_messages.push_back(meta_msg);
  }
  
  // Battery 02 metadata
  String bat02_description = "Battery 02 cell voltage";
  for (int i = 0; i < 32; i++) {
    String meta_msg = "{\"path\":\"electrical.batteries.bat02.cells.cell" + String(i+1) + "\",\"value\":null,\"meta\":{\"units\":\"V\",\"description\":\"" + bat02_description + "\",\"displayName\":\"Cell " + String(i+1) + "\",\"warnLower\":3.0,\"faultLower\":2.8,\"warnUpper\":3.5,\"faultUpper\":3.8}}";
    cell_metadata_messages.push_back(meta_msg);
  }
  
  debugI("Built %d cell metadata messages (in memory, not yet sent)", cell_metadata_messages.size());
}

// Function to send pre-built cell metadata in batches (buffer limit is 20)
// Returns true when all batches are sent
bool send_cell_metadata_batch() {
  const int BATCH_SIZE = 10;  // Send 10 at a time (buffer max is 20)
  int start = metadata_batch_index * BATCH_SIZE;
  int end = min(start + BATCH_SIZE, (int)cell_metadata_messages.size());
  
  for (int i = start; i < end; i++) {
    sensesp_app->get_sk_delta()->append(cell_metadata_messages[i]);
  }
  
  debugI("✓ Sent metadata batch %d (messages %d-%d of %d)", 
         metadata_batch_index + 1, start + 1, end, cell_metadata_messages.size());
  
  metadata_batch_index++;
  
  // Return true if all batches are done
  return (end >= (int)cell_metadata_messages.size());
}

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
  setupThrottle();
  setupLcdDisplay();
  setupKerPropCanBus();
  setupBms();
  setupTempSensors();
  setupFansPumps();
  setupShaftRpm();
  debugD("setup complete");
    
  // Start networking, SK server connections and other SensESP internals
  debugD("Starting sensesp_app->start()");
  debugD("About to call start() - SensESP HTTP server and WiFi AP should initialize here");
  delay(100);
  sensesp_app->start();
  debugD("sensesp_app->start() complete");
  
  // Build cell metadata messages (stored in memory, will send when SK connects)
  build_cell_metadata();
  
  // Send metadata when Signal K websocket is connected, in batches
  app.onRepeat(500, []() {
    // Check actual websocket connection status
    auto ws_client = sensesp_app->get_ws_client();
    if (ws_client && ws_client->is_connected()) {
      if (!was_signalk_connected) {
        debugI("✓ Signal K websocket connected, starting metadata send");
        was_signalk_connected = true;
      }
      // Send next batch if we haven't sent all yet
      if (metadata_batch_index * 10 < (int)cell_metadata_messages.size()) {
        send_cell_metadata_batch();
      }
    }
  });
  
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
  // TODO is this needed or not? delay(2000);
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
    sDisplayData.UpTime = millis() / 1000;  // Use actual system time instead of counting
    // Update shaft RPM display (convert from Hz to RPM for LCD display)
    double frequency = clShaftFreq.getCurrentFrequency();
    sDisplayData.ShaftRpm = (int32_t)(frequency * 60);  // Convert Hz to RPM for display
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
  // Check the CAN bus receiver every 100ms
  app.onRepeat(100, []() { myKerCan.checkReceiver(); });
  // Send idle/neutral mode with zero current to both motors (one time only)
  myKerCan.SendCommands(0, 0, 0);  // 0 current, 0 speed, control mode 0 (neutral/idle)
  debugD("CAN Command sent (Idle/Neutral)");
  
}

// Compound type for battery cell data (like Position in SensESP)
struct BatteryCellsData {
  uint16_t cells[32];      // Individual cell voltages (0-31)
  uint16_t max_voltage;    // Highest cell voltage
  uint16_t min_voltage;    // Lowest cell voltage
  uint16_t max_index;      // Which cell has max voltage
  uint16_t min_index;      // Which cell has min voltage
  uint16_t imbalance;      // Voltage delta (max - min)
};

void setupBms(void) {
  // -------------------------------------------------------------
  // MOD BUS for 2x BMS
  // -------------------------------------------------------------
  
  // Create Signal K outputs for Battery 01 (addr 0x01)
  auto* bat01_voltage_metadata = new SKMetadataWithZones(
      "V", "Battery 01 Voltage", "Forward battery pack voltage");
  bat01_voltage_metadata->add_zone(0, 45, "alarm", "Battery voltage critically low");
  bat01_voltage_metadata->add_zone(45, 48, "warn", "Battery voltage low");
  bat01_voltage_metadata->add_zone(48, 51.2, "normal", "Normal operating voltage");
  bat01_voltage_metadata->add_zone(51.2, 55, "alarm", "Battery voltage overvoltage condition");
  auto* bat01_voltage_output = new SKOutputNumeric<float>(
      "electrical.batteries.bat01.voltage", 
      "/bat01_voltage", 
      bat01_voltage_metadata);
  
  auto* bat01_current_metadata = new SKMetadataWithZones(
      "A", "Battery 01 Current", "Battery 01 charge/discharge current (positive=charging)");
  bat01_current_metadata->add_zone(-200, -150, "alarm", "Excessive discharge current");
  bat01_current_metadata->add_zone(-150, -80, "warn", "High discharge current");
  bat01_current_metadata->add_zone(-80, 80, "normal", "Normal operating current");
  bat01_current_metadata->add_zone(80, 150, "warn", "High charge current");
  bat01_current_metadata->add_zone(150, 200, "alarm", "Excessive charge current");
  auto* bat01_current_output = new SKOutputNumeric<float>(
      "electrical.batteries.bat01.current", 
      "/bat01_current", 
      bat01_current_metadata);
  
  auto* bat01_soc_metadata = new SKMetadataWithZones(
      "ratio", "Battery 01 State of Charge", "Battery 01 state of charge (0.0-1.0)");
  bat01_soc_metadata->add_zone(0, 0.1, "alarm", "Battery critically discharged");
  bat01_soc_metadata->add_zone(0.1, 0.2, "warn", "Battery low");
  bat01_soc_metadata->add_zone(0.2, 0.8, "normal", "Normal operating range");
  bat01_soc_metadata->add_zone(0.8, 1.0, "warn", "Battery nearly full");
  auto* bat01_soc_output = new SKOutputNumeric<float>(
      "electrical.batteries.bat01.capacity.stateOfCharge", 
      "/bat01_soc", 
      bat01_soc_metadata);
  
  auto* bat01_temp_metadata = new SKMetadataWithZones(
      "K", "Battery 01 Temperature", "Battery 01 maximum cell temperature");
  bat01_temp_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  bat01_temp_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  bat01_temp_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - reduce charge/discharge");
  bat01_temp_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  auto* bat01_temp_output = new SKOutputNumeric<float>(
      "electrical.batteries.bat01.temperature", 
      "/bat01_temp", 
      bat01_temp_metadata);
  
  // Create Signal K outputs for Battery 02 (addr 0x02)
  auto* bat02_voltage_metadata = new SKMetadataWithZones(
      "V", "Battery 02 Voltage", "Aft battery pack voltage");
  bat02_voltage_metadata->add_zone(0, 45, "alarm", "Battery voltage critically low");
  bat02_voltage_metadata->add_zone(45, 48, "warn", "Battery voltage low");
  bat02_voltage_metadata->add_zone(48, 51.2, "normal", "Normal operating voltage");
  bat02_voltage_metadata->add_zone(51.2, 55, "alarm", "Battery voltage overvoltage condition");
  auto* bat02_voltage_output = new SKOutputNumeric<float>(
      "electrical.batteries.bat02.voltage", 
      "/bat02_voltage", 
      bat02_voltage_metadata);
  
  auto* bat02_current_metadata = new SKMetadataWithZones(
      "A", "Battery 02 Current", "Battery 02 charge/discharge current (positive=charging)");
  bat02_current_metadata->add_zone(-200, -150, "alarm", "Excessive discharge current");
  bat02_current_metadata->add_zone(-150, -80, "warn", "High discharge current");
  bat02_current_metadata->add_zone(-80, 80, "normal", "Normal operating current");
  bat02_current_metadata->add_zone(80, 150, "warn", "High charge current");
  bat02_current_metadata->add_zone(150, 200, "alarm", "Excessive charge current");
  auto* bat02_current_output = new SKOutputNumeric<float>(
      "electrical.batteries.bat02.current", 
      "/bat02_current", 
      bat02_current_metadata);
  
  auto* bat02_soc_metadata = new SKMetadataWithZones(
      "ratio", "Battery 02 State of Charge", "Battery 02 state of charge (0.0-1.0)");
  bat02_soc_metadata->add_zone(0, 0.1, "alarm", "Battery critically discharged");
  bat02_soc_metadata->add_zone(0.1, 0.2, "warn", "Battery low");
  bat02_soc_metadata->add_zone(0.2, 0.8, "normal", "Normal operating range");
  bat02_soc_metadata->add_zone(0.8, 1.0, "warn", "Battery nearly full");
  auto* bat02_soc_output = new SKOutputNumeric<float>(
      "electrical.batteries.bat02.capacity.stateOfCharge", 
      "/bat02_soc", 
      bat02_soc_metadata);
  
  auto* bat02_temp_metadata = new SKMetadataWithZones(
      "K", "Battery 02 Temperature", "Battery 02 maximum cell temperature");
  bat02_temp_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  bat02_temp_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  bat02_temp_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - reduce charge/discharge");
  bat02_temp_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  auto* bat02_temp_output = new SKOutputNumeric<float>(
      "electrical.batteries.bat02.temperature", 
      "/bat02_temp", 
      bat02_temp_metadata);
  
  BMS_modbus.onData([](uint8_t serverAddress, esp32Modbus::FunctionCode fc,
                       uint16_t address, uint8_t* data, size_t length) {
      // This callback only fires if CRC is VALID
    debugI("✓ DATA RECEIVED (CRC OK) - Addr=0x%04X Slave=0x%02x Length=%d", address, serverAddress, length);
    switch (address) {
      case 0xD000: {
        // Debug: Show raw data for D0000026
        String hex_dump = "RAW D0000 (";
        hex_dump += String(length);
        hex_dump += " bytes): ";
        for (size_t i = 0; i < length && i < 10; i++) {
          hex_dump += String(data[i], HEX);
          hex_dump += " ";
        }
        if (length > 10) hex_dump += "...";
        debugI("%s", hex_dump.c_str());
        
        if (serverAddress == 0x01) {
          Parse_D0000026(&sBMS0_data, data, length);
          debugI("✓ BAT01 D0000 PARSED: Cell[0]=0x%04x Cell[1]=0x%04x VMax=0x%04x VMin=0x%04x\n", 
                 sBMS0_data.wVcellAFE1[0], sBMS0_data.wVcellAFE1[1],
                 sBMS0_data.sVcellStatistics.wVcellMax, sBMS0_data.sVcellStatistics.wVcellMin);
        } else if (serverAddress == 0x02) {
          Parse_D0000026(&sBMS1_data, data, length);
          debugI("✓ BAT02 D0000 PARSED: Cell[0]=0x%04x Cell[1]=0x%04x VMax=0x%04x VMin=0x%04x\n",
                 sBMS1_data.wVcellAFE1[0], sBMS1_data.wVcellAFE1[1],
                 sBMS1_data.sVcellStatistics.wVcellMax, sBMS1_data.sVcellStatistics.wVcellMin);
        }
      } break;
      case 0xD026: {
        // Debug: Show raw data for D026
        String hex_dump = "RAW D026 (";
        hex_dump += String(length);
        hex_dump += " bytes): ";
        for (size_t i = 0; i < length && i < 50; i++) {
          if (data[i] < 0x10) hex_dump += "0";
          hex_dump += String(data[i], HEX);
          hex_dump += " ";
        }
        if (length > 50) hex_dump += "...";
        debugI("%s", hex_dump.c_str());
        
        if (serverAddress == 0x01) {
          Parse_D0260019(&sBMS0_data, data, length);
          debugI("✓ BAT01 D026 PARSED: SOC=%d Icharge=%d Idischarge=%d TempMax=%d",
                 sBMS0_data.sCurrentSocHeatCoolFault.wSOC,
                 sBMS0_data.sCurrentSocHeatCoolFault.wIcharge,
                 sBMS0_data.sCurrentSocHeatCoolFault.wIdischarge,
                 sBMS0_data.sTemperatures.wTempMax);
        } else if (serverAddress == 0x02) {
          Parse_D0260019(&sBMS1_data, data, length);
          debugI("✓ BAT02 D026 PARSED: SOC=%d Icharge=%d Idischarge=%d TempMax=%d",
                 sBMS1_data.sCurrentSocHeatCoolFault.wSOC,
                 sBMS1_data.sCurrentSocHeatCoolFault.wIcharge,
                 sBMS1_data.sCurrentSocHeatCoolFault.wIdischarge,
                 sBMS1_data.sTemperatures.wTempMax);
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
        debugI("D200 response received from slave 0x%02X, length=%d", serverAddress, length);
        if (serverAddress == 0x01) {
          Parse_D2000001(&sBMS0_data, data, length);
          debugI("✓ BAT01 D200 PARSED: wSOC=0x%04X (%d)", 
                 sBMS0_data.sCurrentSocHeatCoolFault.wSOC,
                 sBMS0_data.sCurrentSocHeatCoolFault.wSOC);
        } else if (serverAddress == 0x02) {
          Parse_D2000001(&sBMS1_data, data, length);
          debugI("✓ BAT02 D200 PARSED: wSOC=0x%04X (%d)", 
                 sBMS1_data.sCurrentSocHeatCoolFault.wSOC,
                 sBMS1_data.sCurrentSocHeatCoolFault.wSOC);
        }
      } break;
    }
  });
  BMS_modbus.onError([](esp32Modbus::Error error) {
    const char* error_str = "UNKNOWN";
    switch (error) {
      case esp32Modbus::SUCCES:               error_str = "SUCCESS"; break;
      case esp32Modbus::ILLEGAL_FUNCTION:     error_str = "ILLEGAL_FUNCTION"; break;
      case esp32Modbus::ILLEGAL_DATA_ADDRESS: error_str = "ILLEGAL_DATA_ADDRESS"; break;
      case esp32Modbus::ILLEGAL_DATA_VALUE:   error_str = "ILLEGAL_DATA_VALUE"; break;
      case esp32Modbus::SERVER_DEVICE_FAILURE: error_str = "SERVER_DEVICE_FAILURE"; break;
      case esp32Modbus::ACKNOWLEDGE:          error_str = "ACKNOWLEDGE"; break;
      case esp32Modbus::SERVER_DEVICE_BUSY:   error_str = "SERVER_DEVICE_BUSY"; break;
      case esp32Modbus::NEGATIVE_ACKNOWLEDGE: error_str = "NEGATIVE_ACKNOWLEDGE"; break;
      case esp32Modbus::MEMORY_PARITY_ERROR:  error_str = "MEMORY_PARITY_ERROR"; break;
      case esp32Modbus::TIMEOUT:              error_str = "TIMEOUT"; break;
      case esp32Modbus::INVALID_SLAVE:        error_str = "INVALID_SLAVE"; break;
      case esp32Modbus::INVALID_FUNCTION:     error_str = "INVALID_FUNCTION"; break;
      case esp32Modbus::CRC_ERROR: {
        error_str = "CRC_ERROR";
        debugW("!!! CRC ERROR on response to: Slave=0x%02x Address=0x%04x", 
                      last_requested_slave, last_requested_address);
        debugW("    Device DID respond (we have data), but CRC validation failed.");
        debugW("    This usually means: CRC-16 variant mismatch between library and device");
        break;
      }
      case esp32Modbus::COMM_ERROR:           error_str = "COMM_ERROR"; break;
      default: break;
    }
    debugW("⚠ MODBUS ERROR: 0x%02x (%s)", static_cast<uint8_t>(error), error_str);
  });
  debugD("Initializing Serial1 at 19200 baud, RX=pin 18, TX=pin 17");
  Serial1.begin(19200, SERIAL_8N1, ModBusRxdPin, ModBusTxdPin);  // Modbus connection
  debugD("Calling BMS_modbus.begin()");
  BMS_modbus.begin();
  debugD("BMS_modbus initialized successfully");
  
  // Increase timeout for large responses (D0000 is 76 bytes @ 19200 baud = ~40ms + buffer)
  BMS_modbus.setTimeOutValue(500);  // 500ms timeout should be plenty
  debugD("BMS_modbus timeout set to 500ms");
  
  // Periodic update of battery data to Signal K
  app.onRepeat(1000, [bat01_voltage_output, bat01_current_output, bat01_soc_output, bat01_temp_output,
                      bat02_voltage_output, bat02_current_output, bat02_soc_output, bat02_temp_output]() {
    // Update cell data as compound structure (like Position in SensESP)
    BatteryCellsData bat01_cells = {};
    for (int i = 0; i < 16; i++) bat01_cells.cells[i] = sBMS0_data.wVcellAFE1[i];
    for (int i = 0; i < 16; i++) bat01_cells.cells[16 + i] = sBMS0_data.wVcellAFE2[i];
    bat01_cells.max_voltage = sBMS0_data.sVcellStatistics.wVcellMax;
    bat01_cells.min_voltage = sBMS0_data.sVcellStatistics.wVcellMin;
    bat01_cells.max_index = sBMS0_data.sVcellStatistics.wMaxPosition;
    bat01_cells.min_index = sBMS0_data.sVcellStatistics.wMinPosition;
    bat01_cells.imbalance = sBMS0_data.sVcellStatistics.wVdelta;
    
    BatteryCellsData bat02_cells = {};
    for (int i = 0; i < 16; i++) bat02_cells.cells[i] = sBMS1_data.wVcellAFE1[i];
    for (int i = 0; i < 16; i++) bat02_cells.cells[16 + i] = sBMS1_data.wVcellAFE2[i];
    bat02_cells.max_voltage = sBMS1_data.sVcellStatistics.wVcellMax;
    bat02_cells.min_voltage = sBMS1_data.sVcellStatistics.wVcellMin;
    bat02_cells.max_index = sBMS1_data.sVcellStatistics.wMaxPosition;
    bat02_cells.min_index = sBMS1_data.sVcellStatistics.wMinPosition;
    bat02_cells.imbalance = sBMS1_data.sVcellStatistics.wVdelta;
    
    
    // Emit cell data as individual path entries - values only (metadata sent at startup)
    {
      // Battery 01 cells - each cell as its own path
      for (int i = 0; i < 32; i++) {
        float cell_v;
        if (bat01_cells.cells[i] == 0xEE49) {
          cell_v = -1.0f;  // Invalid/missing data marker
        } else {
          cell_v = bat01_cells.cells[i] / 1000.0f;
        }
        String delta_msg = "{\"path\":\"electrical.batteries.bat01.cells.cell" + String(i+1) + "\",\"value\":" + String(cell_v, 3) + "}";
        sensesp_app->get_sk_delta()->append(delta_msg);
      }
      
      // Battery 02 cells - each cell as its own path
      for (int i = 0; i < 32; i++) {
        float cell_v;
        if (bat02_cells.cells[i] == 0xEE49) {
          cell_v = -1.0f;  // Invalid/missing data marker
        } else {
          cell_v = bat02_cells.cells[i] / 1000.0f;
        }
        String delta_msg = "{\"path\":\"electrical.batteries.bat02.cells.cell" + String(i+1) + "\",\"value\":" + String(cell_v, 3) + "}";
        sensesp_app->get_sk_delta()->append(delta_msg);
      }
    }
    
    // Battery 01 - Forward battery
    if (sBMS0_data.sVcellStatistics.wVbat > 0) {
      bat01_voltage_output->set(sBMS0_data.sVcellStatistics.wVbat / 100.0f);  // Convert 10mV to V
      sDisplayData.ForwardBattery.Voltage = sBMS0_data.sVcellStatistics.wVbat / 10;  // Display in 100mV (512 = 51.2V)
      sDisplayData.ForwardBattery.BmsCommsOk = true;
      
      // Current: positive for charging, negative for discharging
      float bat01_current = (float)sBMS0_data.sCurrentSocHeatCoolFault.wIcharge / 1000.0f;  // Convert mA to A
      if (sBMS0_data.sCurrentSocHeatCoolFault.wIdischarge > 0) {
        bat01_current = -(float)sBMS0_data.sCurrentSocHeatCoolFault.wIdischarge / 1000.0f;  // Discharge is negative
      }
      bat01_current_output->set(bat01_current);
      sDisplayData.ForwardBattery.Current = (int32_t)(bat01_current * 10);  // Display in 100mA units
      
      // SOC: convert from percentage (0-100) to ratio (0.0-1.0)
      debugI("BAT01 SOC raw=%d%% → %.2f", 
             sBMS0_data.sCurrentSocHeatCoolFault.wSOC, 
             sBMS0_data.sCurrentSocHeatCoolFault.wSOC / 100.0f);
      bat01_soc_output->set(sBMS0_data.sCurrentSocHeatCoolFault.wSOC / 100.0f);
      sDisplayData.ForwardBattery.SoC = sBMS0_data.sCurrentSocHeatCoolFault.wSOC;  // Display as percentage
    }
    // Temperature: convert from 0.1°C units with +40°C offset to Kelvin
    if (sBMS0_data.sTemperatures.wTempMax > 0) {
      float temp_celsius = (sBMS0_data.sTemperatures.wTempMax / 10.0f) - 40.0f;  // Remove +40 offset
      bat01_temp_output->set(temp_celsius + 273.15f);
      sDisplayData.ForwardBattery.Temp = (sBMS0_data.sTemperatures.wTempMax / 10) - 40;  // Display in °C
    }
    
    // Battery 02 - Aft battery
    if (sBMS1_data.sVcellStatistics.wVbat > 0) {
      bat02_voltage_output->set(sBMS1_data.sVcellStatistics.wVbat / 100.0f);  // Convert 10mV to V
      sDisplayData.AftBattery.Voltage = sBMS1_data.sVcellStatistics.wVbat / 10;  // Display in 100mV
      sDisplayData.AftBattery.BmsCommsOk = true;
      
      // Current: positive for charging, negative for discharging
      float bat02_current = (float)sBMS1_data.sCurrentSocHeatCoolFault.wIcharge / 1000.0f;  // Convert mA to A
      if (sBMS1_data.sCurrentSocHeatCoolFault.wIdischarge > 0) {
        bat02_current = -(float)sBMS1_data.sCurrentSocHeatCoolFault.wIdischarge / 1000.0f;  // Discharge is negative
      }
      bat02_current_output->set(bat02_current);
      sDisplayData.AftBattery.Current = (int32_t)(bat02_current * 10);  // Display in 100mA units
      
      // SOC: convert from percentage (0-100) to ratio (0.0-1.0)
      bat02_soc_output->set(sBMS1_data.sCurrentSocHeatCoolFault.wSOC / 100.0f);
      sDisplayData.AftBattery.SoC = sBMS1_data.sCurrentSocHeatCoolFault.wSOC;  // Display as percentage
    }
    // Temperature: convert from 0.1°C units with +40°C offset to Kelvin
    if (sBMS1_data.sTemperatures.wTempMax > 0) {
      float temp_celsius = (sBMS1_data.sTemperatures.wTempMax / 10.0f) - 40.0f;  // Remove +40 offset
      bat02_temp_output->set(temp_celsius + 273.15f);
      sDisplayData.AftBattery.Temp = (sBMS1_data.sTemperatures.wTempMax / 10) - 40;  // Display in °C
    }
  });
  
  app.onRepeat(8000, []() {
  //debugD("Sending MOD bus read requests, state_index = %d", state_index);
    switch (state_index) {
      case 0: {
        last_requested_slave = 0x01;
        last_requested_address = 0xD000;
        bool result = BMS_modbus.readHoldingRegisters(0x01, 0xD000, 0x0026);
        state_index++;
        break;
      }
      case 1: {
        last_requested_slave = 0x01;
        last_requested_address = 0xD026;
        bool result = BMS_modbus.readHoldingRegisters(0x01, 0xD026, 0x0019);
        state_index++;
        break;
      }
      case 2: {
        last_requested_slave = 0x01;
        last_requested_address = 0xD100;
        bool result = BMS_modbus.readHoldingRegisters(0x01, 0xD100, 0x0015);
        state_index++;
        break;
      }
      case 3: {
        last_requested_slave = 0x01;
        last_requested_address = 0xD200;
        bool result = BMS_modbus.readHoldingRegisters(0x01, 0xD200, 0x0001);
        state_index = 0;
        break;
      }
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
    ->set_sort_order(100);
   
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
    ->set_sort_order(101);
  
  // Periodic feedback of max motor/controller temp to hysteresis
  app.onRepeat(1000, [&]() {
    // Calculate the max of the 4 current temperatures (in Celsius, convert to Kelvin)
    float temps_celsius[] = {(float)portMotor_temperature, (float)starboardMotor_temperature, 
                             (float)portController_temperature, (float)starboardController_temperature};
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
    ->set_sort_order(102);
} 
void setupShaftRpm(void) {
  // Shaft RPM monitoring - reads pulses from propeller speed sensor via interrupt
  // Updates shaft frequency based on pulse intervals
  
  // Setup RPM input pin and interrupt handler
  pinMode(RpmProxyInputPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(RpmProxyInputPin), isr, RISING);
  
  // Create Signal K output for shaft RPM with metadata (in SI units: Hz)
  auto* shaft_rpm_metadata = new SKMetadataWithZones(
      "Hz",  // units (SI: Hertz)
      "Shaft Speed",  // display_name
      "Propeller shaft rotational speed in Hertz (multiply by 60 for RPM)"  // description
  );
  
  // Configure zones according to Signal K specification (in Hz):
  //   0-16.67 Hz (0-1000 RPM): Nominal (Green)
  shaft_rpm_metadata->add_zone(0, 16.67, "nominal", "Normal operating range");
  //   16.67-20 Hz (1000-1200 RPM): Warning (Yellow)
  shaft_rpm_metadata->add_zone(16.67, 20, "warn", "Approaching maximum");
  //   20-25 Hz (1200-1500 RPM): Alarm (Orange)
  shaft_rpm_metadata->add_zone(20, 25, "alarm", "Exceeding safe operating range");
  //   25+ Hz (1500+ RPM): Fault (Red)
  shaft_rpm_metadata->add_zone(25, "alarm", "Critical - shutdown required");
  
  auto* shaft_rpm_output = new SKOutputNumeric<float>(
      "propulsion.speed", 
      "/shaft_rpm",  // config path for WebUI
      shaft_rpm_metadata
  );
  
  // Periodic update of shaft speed to Signal K and motor runtime tracking
  app.onRepeat(1000, [shaft_rpm_output]() {
    double frequency = clShaftFreq.getCurrentFrequency();
    shaft_rpm_output->set(frequency);  // Send Hz directly (SI units)
    
    // Track motor runtime: start when frequency > 1.67 Hz (~100 RPM), stop when frequency < 0.33 Hz (~20 RPM)
    if (frequency > 1.67 && !motors_running) {
      motors_running = true;
      // debugI("Motors started - beginning runtime accumulation");  // Disabled SK debug output
    } else if (frequency < 0.33 && motors_running) {
      motors_running = false;
      // debugI("Motors stopped - paused runtime accumulation at %lu seconds", motor_runtime_seconds);  // Disabled SK debug output
    }
    
    // Increment runtime when motors are running
    if (motors_running) {
      motor_runtime_seconds++;
      sDisplayData.RunTime = (uint32_t)(motor_runtime_seconds / 60);  // Convert to minutes for display
    }
    
    if (frequency > 0) {
      float rpm = frequency * 60;
      // debugI("Shaft Speed: %.2f Hz (%.0f RPM)", frequency, rpm);  // Disabled SK debug output
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
  
  // Temperature zone thresholds (converted to Kelvin):
  // warn: 40°C = 313.15 K
  // alarm: 70°C = 343.15 K
  // emergency: 90°C = 363.15 K
  
  // Create Signal K outputs with zones for each temperature sensor
  // Port Motor Temperature
  auto* portMotor_metadata = new SKMetadataWithZones(
      "K", "Port Motor Temperature", "Temperature of the port motor", "", -1.0);
  portMotor_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  portMotor_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  portMotor_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - reduce load");
  portMotor_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  auto* portMotor_output = new SKOutputNumeric<float>(
      "propulsion.port.temperature", "/portMotor_temp_sk", portMotor_metadata);
  temp_sensor_portMotor->connect_to(portMotor_output);
  
  // Starboard Motor Temperature
  auto* starboardMotor_metadata = new SKMetadataWithZones(
      "K", "Starboard Motor Temperature", "Temperature of the starboard motor", "", -1.0);
  starboardMotor_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  starboardMotor_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  starboardMotor_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - reduce load");
  starboardMotor_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  auto* starboardMotor_output = new SKOutputNumeric<float>(
      "propulsion.starboard.temperature", "/starboardMotor_temp_sk", starboardMotor_metadata);
  temp_sensor_starboardMotor->connect_to(starboardMotor_output);
  
  // Port Controller Temperature
  auto* portController_metadata = new SKMetadataWithZones(
      "K", "Port Controller Temperature", "Temperature of the port motor controller", "", -1.0);
  portController_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  portController_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  portController_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - reduce load");
  portController_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  auto* portController_output = new SKOutputNumeric<float>(
      "propulsion.port.controller_temperature", "/portController_temp_sk", portController_metadata);
  temp_sensor_portController->connect_to(portController_output);
  
  // Starboard Controller Temperature
  auto* starboardController_metadata = new SKMetadataWithZones(
      "K", "Starboard Controller Temperature", "Temperature of the starboard motor controller", "", -1.0);
  starboardController_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  starboardController_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  starboardController_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - reduce load");
  starboardController_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  auto* starboardController_output = new SKOutputNumeric<float>(
      "propulsion.starboard.controller_temperature", "/starboardController_temp_sk", starboardController_metadata);
  temp_sensor_starboardController->connect_to(starboardController_output);
  
  // Engine Room Temperature
  auto* engineRoom_metadata = new SKMetadataWithZones(
      "K", "Engine Room Temperature", "Ambient temperature in the engine room", "", -1.0);
  engineRoom_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  engineRoom_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  engineRoom_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - check cooling");
  engineRoom_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  auto* engineRoom_output = new SKOutputNumeric<float>(
      "environment.engineRoom.temperature", "/engineRoom_temp_sk", engineRoom_metadata);
  temp_sensor_engineRoom->connect_to(engineRoom_output);
  
  // Coolant Temperature
  auto* coolant_metadata = new SKMetadataWithZones(
      "K", "Coolant Temperature", "Temperature of the engine coolant", "", -1.0);
  coolant_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  coolant_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  coolant_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - check cooling system");
  coolant_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  auto* coolant_output = new SKOutputNumeric<float>(
      "environment.coolant.temperature", "/coolant_temp_sk", coolant_metadata);
  temp_sensor_coolant->connect_to(coolant_output);
  
  // Config Items for the UI
  ConfigItem(temp_sensor_portMotor)
    ->set_title("Port Motor Temperature Sensor")
    ->set_description("Temperature sensor for the port motor.")
    ->set_sort_order(200);
  ConfigItem(temp_sensor_starboardMotor)
    ->set_title("Starboard Motor Temperature Sensor")
    ->set_description("Temperature sensor for the starboard motor.")
    ->set_sort_order(201);
  ConfigItem(temp_sensor_portController)
    ->set_title("Port Controller Temperature Sensor")
    ->set_description("Temperature sensor for the port motor controller.")
    ->set_sort_order(202);
  ConfigItem(temp_sensor_starboardController)
    ->set_title("Starboard Controller Temperature Sensor")
    ->set_description("Temperature sensor for the starboard motor controller.")
    ->set_sort_order(203);
  ConfigItem(temp_sensor_engineRoom)
    ->set_title("Engine Room Temperature Sensor")
    ->set_description("Temperature sensor for the engine room ambient temperature.")
    ->set_sort_order(204);
  ConfigItem(temp_sensor_coolant)
    ->set_title("Coolant Temperature Sensor")
    ->set_description("Temperature sensor for the motor coolant temperature.")
    ->set_sort_order(205);


  // Connect temperature sensors to global variables and Signal K outputs
  // Create Signal K outputs
  auto* port_motor_sk = new SKOutputFloat("propulsion.port.motor.temperature");
  auto* starboard_motor_sk = new SKOutputFloat("propulsion.starboard.motor.temperature");
  auto* port_ctrl_sk = new SKOutputFloat("propulsion.port.controller.temperature");
  auto* starboard_ctrl_sk = new SKOutputFloat("propulsion.starboard.controller.temperature");
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
  // Configure ADC for analog throttle reading
  analogSetAttenuation(ADC_11db);  // Full range 0-3.3V
  
  // Create String variables for config paths
  String path_adc_ref = "/throttle/adc_reference_voltage";
  String path_adc_hyst = "/throttle/adc_neutral_hysteresis";
  String path_idle_thresh = "/throttle/idle_threshold";
  String path_idle_pct = "/throttle/idle_percent";
  String path_max_current = "/throttle/max_phase_current";
  
  // Initialize NumberConfig for throttle parameters
  config_adc_reference_voltage = new NumberConfig(adc_reference_voltage, path_adc_ref);
  config_adc_neutral_hysteresis = new NumberConfig(adc_neutral_hysteresis, path_adc_hyst);
  config_throttle_idle_threshold = new NumberConfig(throttle_idle_threshold, path_idle_thresh);
  config_throttle_idle_percent = new NumberConfig(throttle_idle_percent, path_idle_pct);
  config_max_phase_current = new NumberConfig(max_phase_current, path_max_current);
  
  // Register NumberConfig items in the UI
  ConfigItem(config_adc_reference_voltage);
  ConfigItem(config_adc_neutral_hysteresis);
  ConfigItem(config_throttle_idle_threshold);
  ConfigItem(config_throttle_idle_percent);
  ConfigItem(config_max_phase_current);
  
  // Create Signal K output for ADC throttle input
  auto* adc_throttle_output = new SKOutputNumeric<float>(
      "propulsion.helm.throttlePosition",
      "/adc_throttle",
      new SKMetadata("ratio", "ADC Throttle", "Throttle lever position from ADC potentiometer (0-100%)")
  );
  
  // Update adc_midpoint_voltage based on current reference voltage
  adc_midpoint_voltage = adc_reference_voltage / 2.0f;
  
  // Read ADC throttle and send commands to both motors
  app.onRepeat(100, [adc_throttle_output]() {
    // Read ADC throttle (12-bit: 0-4095 maps to 0-5.0V reference)
    uint16_t raw_adc = analogRead(kAnalogInputPin);
    // Calculate voltage from raw ADC value
    float voltage = (raw_adc / (float)ADC_MAX_VALUE) * adc_reference_voltage;
    
    // Update midpoint based on current reference voltage
    adc_midpoint_voltage = adc_reference_voltage / 2.0f;
    
    // Calculate bidirectional throttle percentage with neutral hysteresis
    float throttle_percent = 0.0f;
    float neutral_lower = adc_midpoint_voltage - (adc_neutral_hysteresis / 2.0f);
    float neutral_upper = adc_midpoint_voltage + (adc_neutral_hysteresis / 2.0f);
    
    if (voltage < neutral_lower) {
      // Reverse direction (below neutral) - negative throttle
      // Maps neutral_lower = 0%, 0V = -100%
      throttle_percent = ((neutral_lower - voltage) / neutral_lower);
    } else if (voltage > neutral_upper) {
      // Forward direction (above neutral) - positive throttle
      // Maps neutral_upper = 0%, adc_reference_voltage = 100%
      throttle_percent = ((voltage - neutral_upper) / (adc_reference_voltage - neutral_upper));
    }
    // else: within neutral hysteresis band, throttle_percent remains 0.0f
    
    // Clamp to -100 to +100
    if (throttle_percent > 1.0f) throttle_percent = 1.0f;
    if (throttle_percent < -1.0f) throttle_percent = -1.0f;
    
    // Apply idle mode: if throttle is between -IDLE_THRESHOLD and +IDLE_THRESHOLD, set to idle throttle
    if (throttle_percent > 0.0f && throttle_percent < throttle_idle_threshold) {
      throttle_percent = throttle_idle_percent;
    } else if (throttle_percent < 0.0f && throttle_percent > -throttle_idle_threshold) {
      throttle_percent = -throttle_idle_percent;
    }
    
    // Output to Signal K
    adc_throttle_output->set(throttle_percent);
    
    // Send throttle command to both motors (Port and Starboard)
    // Convert throttle percentage to target current using max phase current variable
    int16_t target_current = (int16_t)((throttle_percent) * max_phase_current);
    
    // Send to Port motor
    myKerCan.McuPort.SendCommand(target_current, 0, 0);
    // Send to Starboard motor
    myKerCan.McuStarboard.SendCommand(target_current, 0, 0);
    
    // Debug: show raw ADC value (12-bit: 0-4095), calculated voltage, and throttle
    //ådebugD("ADC pin=%d raw=0x%04X (%d/4095) voltage=%.2fV Throttle: %.1f%% -> Current: %d A", kAnalogInputPin, raw_adc, raw_adc, voltage, throttle_percent, target_current);
  });
  
  // =========================================================
  // THROTTLE FEEDBACK - From both motor controllers
  // =========================================================
  
 /*  // Create Signal K outputs for both motor throttle feedback
  auto* port_throttle_fb = new SKOutputNumeric<float>(
      "propulsion.port.throttlePosition",
      "/port_throttle_fb",
      new SKMetadata("ratio", "Port Motor Throttle", "Throttle feedback from Port motor controller")
  );
  
  auto* starboard_throttle_fb = new SKOutputNumeric<float>(
      "propulsion.starboard.throttlePosition",
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
  */
}
