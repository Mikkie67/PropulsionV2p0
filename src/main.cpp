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
#include <ArduinoJson.h>
#include <memory>

#include "bms_setup.h"
#include "ker_can.hpp"
#include "propDisplay.h"
#include "rpm_class.hpp"
#include "sensesp.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_delta_queue.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/ui/ui_controls.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "signalk_extended_metadata.h"
#include <sensesp/transforms/hysteresis.h>

using namespace sensesp;
using namespace sensesp::onewire;

void setupLcdDisplay(void);
void setupKerPropCanBus(void);
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
const byte OneWirePin = 37;                      // KerProp.1Wire
const uint16_t TEMP_SENSOR_READ_INTERVAL = 1000; // Temperature sensor read interval in milliseconds
// -----------------------------------------------------------------------------
// Opto inputs for status of fans and pumps
// -----------------------------------------------------------------------------
const byte CoolantFanStatus = 39;  // KerProp.DigIn1
const byte CoolantPumpStatus = 40; // KerProp.DigIn2
const byte AmbientFanStatus = 41;  // KerProp.DigIn3
const byte RpmProxyInputPin = 42;  // KerProp.RpmProxy
const byte SpareOptoInput5 = 45;   // KerProp.DigIn5
const byte SpareOptoInput6 = 46;   // KerProp.DigIn6
const byte SpareOptoInput7 = 47;   // KerProp.DigIn7
const byte SpareOptoInput8 = 48;   // KerProp.DigIn8
// -----------------------------------------------------------------------------
// Solid State Relay outputs to control relays for fans and pumps
// -----------------------------------------------------------------------------
const byte CoolantFanControl = 9;   // KerPRop.RelayControl1
const byte CoolantPumpControl = 10; // KerProp.RelayControl2
const byte AmbientFanControl = 11;  // KerProp.RelayControl3
const byte SpareControl1 = 12;      // KerProp.RelayControl4
const byte SpareControl2 = 13;      // KerProp.RelayControl5
const byte SpareControl3 = 14;      // KerProp.RelayControl6

// -----------------------------------------------------------------------------
// CAN defines --> using TWAI controller
// -----------------------------------------------------------------------------
const byte CAN_TX = 16; // KerProp.CAN_TXD
const byte CAN_RX = 15; // KerProp.CAN_RXD
const byte CAN_CLK = -1;
const byte CAN_BUS_OFF = -1;
ker_can myKerCan(239, 240);
// -----------------------------------------------------------------------------
//  Analog (Throttle)
// -----------------------------------------------------------------------------
const uint8_t kAnalogInputPin = 2; // KerProp.ADC_IN2 (GPIO2)
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
float throttle_idle_threshold = 0.10f; // % (absolute value)
// Throttle percentage to use when in idle mode
float throttle_idle_percent = 0.10f; // %
// Maximum phase current for motor control (in Amps)
float max_phase_current = 250.0f; // A

// NumberConfig objects for persistent storage and UI configuration
NumberConfig* config_adc_reference_voltage = nullptr;
NumberConfig* config_adc_neutral_hysteresis = nullptr;
NumberConfig* config_throttle_idle_threshold = nullptr;
NumberConfig* config_throttle_idle_percent = nullptr;
NumberConfig* config_max_phase_current = nullptr;
// -----------------------------------------------------------------------------
//  LCD
// -----------------------------------------------------------------------------
const byte LCD_CS = 7;    // KerProp.LCD_CS
const byte LCD_DC = 6;    // KerProp.LCD_DC
const byte LCD_MOSI = 35; // KerProp.LCD_MOSI
const byte LCD_CLK = 0;   // KerProp.LCD_CLK
const byte LCD_RST = 5;   // KerProp.LCD_RST
const byte LCD_MISO = 36; // KerProp.LCD_MISO
const byte LCD_LED = 4;   // KerProp.LCD_LED
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
bool coolant_fan_state = false;  // Current state of coolant fan relay
bool coolant_pump_state = false; // Current state of coolant pump relay
bool ambient_fan_state = false;  // Current state of ambient fan relay
bool spare1_relay_state = false; // Current state of spare1 relay
bool spare2_relay_state = false; // Current state of spare2 relay
bool spare3_relay_state = false; // Current state of spare3 relay

// Motor runtime tracking (persistent across sessions)
bool motors_running = false;             // Current motor running state
unsigned long motor_runtime_seconds = 0; // Total accumulated motor runtime

// Coolant pump max temperature tracking - updated by sensor callbacks
float max_motor_controller_temp_kelvin = 273.15f; // Track max of 4 temps in Kelvin for pump control

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
auto coolant_fan_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>([](bool on) {
  // drive your GPIO / relay here
  digitalWrite(CoolantFanControl, on ? HIGH : LOW);
  if (on != coolant_fan_state) {
    coolant_fan_state = on;
    sDisplayData.CoolantFan = on;
    debugD("Coolant fan command: %s", on ? "ON" : "OFF");
  }
});
auto ambient_fan_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>([](bool on) {
  // drive your GPIO / relay here
  digitalWrite(AmbientFanControl, on ? HIGH : LOW);
  if (on != ambient_fan_state) {
    ambient_fan_state = on;
    sDisplayData.AmbientFan = on;
    debugD("Ambient fan command: %s", on ? "ON" : "OFF");
  }
});
auto coolant_pump_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>([](bool on) {
  // drive your GPIO / relay here
  digitalWrite(CoolantPumpControl, on ? HIGH : LOW);
  if (on != coolant_pump_state) {
    coolant_pump_state = on;
    sDisplayData.CoolantPump = on;
    debugD("Coolant pump command: %s", on ? "ON" : "OFF");
  }
});
auto spare1_relay_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>([](bool on) {
  // drive your GPIO / relay here
  digitalWrite(SpareControl1, on ? HIGH : LOW);
  if (on != spare1_relay_state) {
    spare1_relay_state = on;
    debugD("Spare1 relay command: %s", on ? "ON" : "OFF");
  }
});
auto spare2_relay_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>([](bool on) {
  // drive your GPIO / relay here
  digitalWrite(SpareControl2, on ? HIGH : LOW);
  if (on != spare2_relay_state) {
    spare2_relay_state = on;
    debugD("Spare2 relay command: %s", on ? "ON" : "OFF");
  }
});
auto spare3_relay_cmd = std::make_shared<sensesp::LambdaConsumer<bool>>([](bool on) {
  // drive your GPIO / relay here
  digitalWrite(SpareControl3, on ? HIGH : LOW);
  if (on != spare3_relay_state) {
    spare3_relay_state = on;
    debugD("Spare3 relay command: %s", on ? "ON" : "OFF");
  }
});
// ----------------------------------------------------------
// Interrupt service routine
// ----------------------------------------------------------
// Timestamp of the last RPM pulse, in microseconds
volatile unsigned long lastTime = 0;
// Timestamp of the current RPM pulse, in microseconds
volatile unsigned long currentTime = 0;
// RPM value
volatile float shaftHz = 0;
volatile unsigned long timeDifference = 0;
volatile bool updateRpm = false;
ICACHE_RAM_ATTR void isr() {
  lastTime = micros();
  updateRpm = true;
}

void send_cell_metadata_example() {
  // Only initialize zones and SKOutputNumeric once
  auto* bat01_cell_metadata = new signalk_extended_metadata("V", "Cell 1", "Battery 01 cell voltage");
  bat01_cell_metadata->add_zone(2.6f, NAN, String("fault"), String("Cell voltage critically low"));
  bat01_cell_metadata->add_zone(2.9f, NAN, String("warn"), String("Cell voltage low"));
  bat01_cell_metadata->add_zone(NAN, 3.55f, String("warn"), String("Cell voltage high"));
  bat01_cell_metadata->add_zone(NAN, 3.65f, String("fault"), String("Cell voltage critically high"));
  auto* bat01_cells_output =
      new SKOutputNumeric<float>("electrical.batteries.bat01.cells.cell1", "/bat01_cell1", bat01_cell_metadata);

  auto* bat01_voltage_metadata =
      new signalk_extended_metadata("V", "Battery 01 Voltage", "Forward battery pack voltage");
  bat01_voltage_metadata->add_zone(0, 45, "alarm", "Battery voltage critically low");
  bat01_voltage_metadata->add_zone(45, 48, "warn", "Battery voltage low");
  bat01_voltage_metadata->add_zone(48, 55, "normal", "Normal operating voltage");
  bat01_voltage_metadata->add_zone(55, 60, "alarm", "Battery voltage overvoltage condition");
  auto* bat01_voltage_output =
      new SKOutputNumeric<float>("electrical.batteries.bat01.voltage", "/bat01_voltage", bat01_voltage_metadata);

  app.onRepeat(1000, [bat01_cells_output, bat01_voltage_output]() {
    debugD("Updating cell voltage and sending SK delta with metadata zones");
    bat01_cells_output->set(10.0f);   // Example value to trigger zones
    bat01_voltage_output->set(50.0f); // Example value to trigger zones
    });
  // });
}
// Build and send a standards-compliant Signal K metadata delta for a cell
void send_cell_metadata_delta_sk() {
  // Use ArduinoJson's DynamicJsonDocument for runtime allocation
  DynamicJsonDocument doc(1024);
  JsonObject root = doc.to<JsonObject>();
  JsonArray updates = root.createNestedArray("updates");
  JsonObject update = updates.add<JsonObject>();
  JsonArray meta = update.createNestedArray("meta");
  JsonObject meta_entry = meta.add<JsonObject>();
  meta_entry["path"] = "electrical.batteries.bat01.cells.cell1";
  JsonObject value_obj = meta_entry.createNestedObject("value");
  value_obj["units"] = "V";
  value_obj["description"] = "Battery 01 cell voltage";
  value_obj["displayName"] = "Cell 1";
  JsonArray zones_arr = value_obj.createNestedArray("zones");
  JsonObject z1 = zones_arr.add<JsonObject>();
  z1["lower"] = 2.6;
  z1["state"] = "fault";
  z1["message"] = "Cell voltage critically low";
  JsonObject z2 = zones_arr.add<JsonObject>();
  z2["lower"] = 2.9;
  z2["state"] = "warn";
  z2["message"] = "Cell voltage low";
  JsonObject z3 = zones_arr.add<JsonObject>();
  z3["upper"] = 3.55;
  z3["state"] = "warn";
  z3["message"] = "Cell voltage high";
  JsonObject z4 = zones_arr.add<JsonObject>();
  z4["upper"] = 3.65;
  z4["state"] = "fault";
  z4["message"] = "Cell voltage critically high";
  String metadata_delta;
  serializeJson(root, metadata_delta);
  debugD("SK metadata delta: %s", metadata_delta.c_str());
  sensesp_app->get_sk_delta()->append(metadata_delta);
}

// ----------------------------------------------------------
// The setup function performs one-time application initialization.
// ----------------------------------------------------------
void setup() {
  SetupLogging(ESP_LOG_DEBUG);
  File root, file; // Declare SPIFFS file variables

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
  // -------------------------------------------------------------
  // Check SPIFFS files AFTER SensESP initializes filesystem
  // -------------------------------------------------------------
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
  // -------------------------------------------------------------
  // IVOR setupThrottle();
  setupLcdDisplay();
  // IVOR setupKerPropCanBus();
  setupBms(sDisplayData);
  // IVOR setupTempSensors();
  // IVOR setupFansPumps();
  // IVOR setupShaftRpm();
  debugD("setup complete");

   // Send metadata when Signal K websocket is connected, in batches
  app.onRepeat(500, []() {
    // Check actual websocket connection status
    auto ws_client = sensesp_app->get_ws_client();
    if (ws_client && ws_client->is_connected()) {
      if (!was_signalk_connected) {
        debugI("✓ Signal K websocket connected, starting metadata send");
        was_signalk_connected = true;
        // send_cell_metadata_delta_sk()
        
        // send_cell_metadata_example();

        // Always rebuild metadata on new connection
        // build_all_bms_metadata();
        metadata_batch_index = 0;
      }
      // Send next batch if we haven't sent all yet
      if (metadata_batch_index * 10 < (int)cell_metadata_messages.size()) {
        // send_cell_metadata_batch();
      }
    } else {
      was_signalk_connected = false;
    }
  });
 // send_cell_metadata_example();


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
    sDisplayData.UpTime = millis() / 1000; // Use actual system time instead of counting
    // Update shaft RPM display (convert from Hz to RPM for LCD display)
    double frequency = clShaftFreq.getCurrentFrequency();
    sDisplayData.ShaftRpm = (int32_t)(frequency * 60); // Convert Hz to RPM for display
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
  myKerCan.SendCommands(0, 0, 0); // 0 current, 0 speed, control mode 0 (neutral/idle)
  debugD("CAN Command sent (Idle/Neutral)");
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
  coolant_fan_hyst =
      std::make_shared<sensesp::Hysteresis<float, bool>>(303.15f, // lower threshold (OFF) in Kelvin (30°C)
                                                         305.15f, // upper threshold (ON) in Kelvin (32°C)
                                                         false,   // low_output
                                                         true,    // high_output
                                                         "/coolant_fan_control");

  temp_sensor_coolant->connect_to(coolant_fan_hyst)->connect_to(coolant_fan_cmd);
  ConfigItem(coolant_fan_hyst.get())
      ->set_title("Coolant Fan Control")
      ->set_description("Hysteresis control with ON/OFF thresholds in Kelvin (add 273.15 to convert from Celsius).")
      ->set_sort_order(100);

  // =========================================================
  // COOLANT PUMP - ON if ANY motor/controller exceeds threshold, OFF when ALL below
  // =========================================================
  // Initialize hysteresis transform for coolant pump control using max motor/controller temp
  coolant_pump_hyst =
      std::make_shared<sensesp::Hysteresis<float, bool>>(313.15f, // lower threshold (OFF) in Kelvin (40°C)
                                                         315.15f, // upper threshold (ON) in Kelvin (42°C)
                                                         false,   // low_output
                                                         true,    // high_output
                                                         "/coolant_pump_control");

  coolant_pump_hyst->connect_to(coolant_pump_cmd);
  ConfigItem(coolant_pump_hyst.get())
      ->set_title("Coolant Pump Control")
      ->set_description("Hysteresis control based on max motor/controller temperature. ON/OFF thresholds in Kelvin "
                        "(add 273.15 to convert from Celsius).")
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
  ambient_fan_hyst =
      std::make_shared<sensesp::Hysteresis<float, bool>>(308.15f, // lower threshold (OFF) in Kelvin (35°C)
                                                         310.15f, // upper threshold (ON) in Kelvin (37°C)
                                                         false,   // low_output
                                                         true,    // high_output
                                                         "/ambient_fan_control");

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
  signalk_extended_metadata* shaft_rpm_metadata =
      new signalk_extended_metadata("Hz",          // units (SI: Hertz)
                                    "Shaft Speed", // display_name
                                    "Propeller shaft rotational speed in Hertz (multiply by 60 for RPM)" // description
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

  SKOutputNumeric<float>* shaft_rpm_output = new SKOutputNumeric<float>("propulsion.speed",
                                                                        "/shaft_rpm", // config path for WebUI
                                                                        shaft_rpm_metadata);

  // Periodic update of shaft speed to Signal K and motor runtime tracking
  app.onRepeat(1000, [shaft_rpm_output]() {
    double frequency = clShaftFreq.getCurrentFrequency();
    shaft_rpm_output->set(frequency); // Send Hz directly (SI units)

    // Track motor runtime: start when frequency > 1.67 Hz (~100 RPM), stop when frequency < 0.33 Hz (~20 RPM)
    if (frequency > 1.67 && !motors_running) {
      motors_running = true;
      // debugI("Motors started - beginning runtime accumulation");  // Disabled SK debug output
    } else if (frequency < 0.33 && motors_running) {
      motors_running = false;
      // debugI("Motors stopped - paused runtime accumulation at %lu seconds", motor_runtime_seconds);  // Disabled SK
      // debug output
    }

    // Increment runtime when motors are running
    if (motors_running) {
      motor_runtime_seconds++;
      sDisplayData.RunTime = (uint32_t)(motor_runtime_seconds / 60); // Convert to minutes for display
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
  temp_sensor_starboardController =
      new OneWireTemperature(dts, TEMP_SENSOR_READ_INTERVAL, "/2_oneWire/starboardControllerTemp");
  temp_sensor_engineRoom = new OneWireTemperature(dts, TEMP_SENSOR_READ_INTERVAL, "/2_oneWire/EngineRoomTemp");
  temp_sensor_coolant = new OneWireTemperature(dts, TEMP_SENSOR_READ_INTERVAL, "/2_oneWire/CoolantTemp");

  debugI("OneWire sensors detected on pin %d", OneWirePin);

  // Temperature zone thresholds (converted to Kelvin):
  // warn: 40°C = 313.15 K
  // alarm: 70°C = 343.15 K
  // emergency: 90°C = 363.15 K

  // Create Signal K outputs with zones for each temperature sensor
  // Port Motor Temperature
  signalk_extended_metadata* portMotor_metadata =
      new signalk_extended_metadata("K", "Port Motor Temperature", "Temperature of the port motor", "", -1.0);
  portMotor_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  portMotor_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  portMotor_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - reduce load");
  portMotor_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  SKOutputNumeric<float>* portMotor_output =
      new SKOutputNumeric<float>("propulsion.port.temperature", "/portMotor_temp_sk", portMotor_metadata);
  temp_sensor_portMotor->connect_to(portMotor_output);

  // Starboard Motor Temperature
  signalk_extended_metadata* starboardMotor_metadata =
      new signalk_extended_metadata("K", "Starboard Motor Temperature", "Temperature of the starboard motor", "", -1.0);
  starboardMotor_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  starboardMotor_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  starboardMotor_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - reduce load");
  starboardMotor_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  SKOutputNumeric<float>* starboardMotor_output = new SKOutputNumeric<float>(
      "propulsion.starboard.temperature", "/starboardMotor_temp_sk", starboardMotor_metadata);
  temp_sensor_starboardMotor->connect_to(starboardMotor_output);

  // Port Controller Temperature
  signalk_extended_metadata* portController_metadata = new signalk_extended_metadata(
      "K", "Port Controller Temperature", "Temperature of the port motor controller", "", -1.0);
  portController_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  portController_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  portController_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - reduce load");
  portController_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  SKOutputNumeric<float>* portController_output = new SKOutputNumeric<float>(
      "propulsion.port.controller_temperature", "/portController_temp_sk", portController_metadata);
  temp_sensor_portController->connect_to(portController_output);

  // Starboard Controller Temperature
  signalk_extended_metadata* starboardController_metadata = new signalk_extended_metadata(
      "K", "Starboard Controller Temperature", "Temperature of the starboard motor controller", "", -1.0);
  starboardController_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  starboardController_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  starboardController_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - reduce load");
  starboardController_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  SKOutputNumeric<float>* starboardController_output = new SKOutputNumeric<float>(
      "propulsion.starboard.controller_temperature", "/starboardController_temp_sk", starboardController_metadata);
  temp_sensor_starboardController->connect_to(starboardController_output);

  // Engine Room Temperature
  signalk_extended_metadata* engineRoom_metadata =
      new signalk_extended_metadata("K", "Engine Room Temperature", "Ambient temperature in the engine room", "", -1.0);
  engineRoom_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  engineRoom_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  engineRoom_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - check cooling");
  engineRoom_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  SKOutputNumeric<float>* engineRoom_output =
      new SKOutputNumeric<float>("environment.engineRoom.temperature", "/engineRoom_temp_sk", engineRoom_metadata);
  temp_sensor_engineRoom->connect_to(engineRoom_output);

  // Coolant Temperature
  signalk_extended_metadata* coolant_metadata =
      new signalk_extended_metadata("K", "Coolant Temperature", "Temperature of the engine coolant", "", -1.0);
  coolant_metadata->add_zone(0, 313.15, "normal", "Normal operating range");
  coolant_metadata->add_zone(313.15, 343.15, "warn", "Approaching thermal limit");
  coolant_metadata->add_zone(343.15, 363.15, "alarm", "Thermal alarm - check cooling system");
  coolant_metadata->add_zone(363.15, "emergency", "Critical temperature - shutdown required");
  SKOutputNumeric<float>* coolant_output =
      new SKOutputNumeric<float>("environment.coolant.temperature", "/coolant_temp_sk", coolant_metadata);
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
  temp_sensor_portMotor->connect_to(new LambdaConsumer<float>([port_motor_sk](const float& temp) {
    float celsius = temp - 273.15f;
    portMotor_temperature = celsius;
    sDisplayData.Port.MotorTemp = (int)celsius;
    port_motor_sk->set(temp);
  }));

  // Starboard Motor Temperature
  temp_sensor_starboardMotor->connect_to(new LambdaConsumer<float>([starboard_motor_sk](const float& temp) {
    float celsius = temp - 273.15f;
    starboardMotor_temperature = celsius;
    sDisplayData.Starboard.MotorTemp = (int)celsius;
    starboard_motor_sk->set(temp);
  }));

  // Port Controller Temperature
  temp_sensor_portController->connect_to(new LambdaConsumer<float>([port_ctrl_sk](const float& temp) {
    float celsius = temp - 273.15f;
    portController_temperature = celsius;
    sDisplayData.Port.ControllerTemp = (int)celsius;
    port_ctrl_sk->set(temp);
  }));

  // Starboard Controller Temperature
  temp_sensor_starboardController->connect_to(new LambdaConsumer<float>([starboard_ctrl_sk](const float& temp) {
    float celsius = temp - 273.15f;
    starboardController_temperature = celsius;
    sDisplayData.Starboard.ControllerTemp = (int)celsius;
    starboard_ctrl_sk->set(temp);
  }));

  // Engine Room Temperature
  temp_sensor_engineRoom->connect_to(new LambdaConsumer<float>([engine_room_sk](const float& temp) {
    float celsius = temp - 273.15f;
    engineroom_temperature = celsius;
    sDisplayData.AmbientTemp = (int)celsius;
    engine_room_sk->set(temp);
  }));

  // Coolant Temperature
  temp_sensor_coolant->connect_to(new LambdaConsumer<float>([coolant_sk](const float& temp) {
    float celsius = temp - 273.15f;
    coolant_temperature = celsius;
    sDisplayData.CoolantTemp = (int)celsius;
    coolant_sk->set(temp);
  }));
}
void setupThrottle(void) {
  // Configure ADC for analog throttle reading
  analogSetAttenuation(ADC_11db); // Full range 0-3.3V

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
      "propulsion.helm.throttlePosition", "/adc_throttle",
      new SKMetadata("ratio", "ADC Throttle", "Throttle lever position from ADC potentiometer (0-100%)"));

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
    if (throttle_percent > 1.0f)
      throttle_percent = 1.0f;
    if (throttle_percent < -1.0f)
      throttle_percent = -1.0f;

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
    int16_t target_current = (int16_t)((throttle_percent)*max_phase_current);

    // Send to Port motor
    myKerCan.McuPort.SendCommand(target_current, 0, 0);
    // Send to Starboard motor
    myKerCan.McuStarboard.SendCommand(target_current, 0, 0);

    // Debug: show raw ADC value (12-bit: 0-4095), calculated voltage, and throttle
    // ådebugD("ADC pin=%d raw=0x%04X (%d/4095) voltage=%.2fV Throttle: %.1f%% -> Current: %d A", kAnalogInputPin,
    // raw_adc, raw_adc, voltage, throttle_percent, target_current);
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
