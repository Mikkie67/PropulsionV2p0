#include "bms_setup.h"

#include "sensesp.h"
#include "sensesp/signalk/signalk_delta_queue.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app.h"
#include "signalk_extended_metadata.h"

using namespace sensesp;

// Pin definitions for Modbus
static const byte ModBusDePin = 8;   // KerProp.MOD_DE
static const byte ModBusTxdPin = 17; // KerProp.MOD_TXD
static const byte ModBusRxdPin = 18; // KerProp.MOD_RXD

// Global BMS data and Modbus instance
esp32ModbusRTU BMS_modbus(&Serial1, ModBusTxdPin, ModBusRxdPin, ModBusDePin);

// Modbus state tracking
static uint16_t last_requested_address = 0;
static uint8_t last_requested_slave = 0;
static uint8_t state_index = 0;

// Cell metadata for Signal K
std::vector<String> cell_metadata_messages;
int metadata_batch_index = 0;
bool was_signalk_connected = false;

// External reference to ReactESP event loop (defined in main.cpp)
extern reactesp::ReactESP app;

// Move cell outputs to file/global scope so they can be used in lambdas
std::vector<SKOutputNumeric<float>*> bat01_cell_outputs;
std::vector<SKOutputNumeric<float>*> bat02_cell_outputs;

std::vector<SKOutputNumeric<int>*> bat01_bnAFE1_outputs;
std::vector<SKOutputNumeric<int>*> bat01_bnAFE2_outputs;

std::vector<SKOutputNumeric<int>*> bat02_bnAFE1_outputs;
std::vector<SKOutputNumeric<int>*> bat02_bnAFE2_outputs;

std::vector<SKOutputNumeric<float>*> bat01_temp_outputs;
std::vector<SKOutputNumeric<float>*> bat02_temp_outputs;

std::vector<SKOutputNumeric<float>*> bat01_temp_env_outputs;
std::vector<SKOutputNumeric<float>*> bat02_temp_env_outputs;

// Battery 01 SKOutputNumeric outputs (global)
SKOutputNumeric<float>* bat01_max_cell_output;
SKOutputNumeric<float>* bat01_min_cell_output;
SKOutputNumeric<float>* bat01_max_position_output;
SKOutputNumeric<float>* bat01_min_position_output;
SKOutputNumeric<float>* bat01_vdelta_output;
SKOutputNumeric<float>* bat01_pack_voltage_output;
SKOutputNumeric<float>* bat01_temp_mos_output;
SKOutputNumeric<float>* bat01_temp_max_output;
SKOutputNumeric<float>* bat01_temp_min_output;
SKOutputNumeric<float>* bat01_current_output;
SKOutputNumeric<float>* bat01_soc_output;
SKOutputNumeric<float>* bat01_soh_output;
SKOutputNumeric<float>* bat01_resma_output;
SKOutputNumeric<float>* bat01_fullma_output;
SKOutputNumeric<float>* bat01_facma_output;
SKOutputNumeric<float>* bat01_cycletime_output;
SKOutputNumeric<float>* bat01_faultfirst_output;
SKOutputNumeric<float>* bat01_faultsecond_output;
SKOutputNumeric<float>* bat01_faultthird_output;
SKOutputNumeric<float>* bat01_heatcoolerror_output;

// Battery 02 SKOutputNumeric outputs (global)
SKOutputNumeric<float>* bat02_max_cell_output;
SKOutputNumeric<float>* bat02_min_cell_output;
SKOutputNumeric<float>* bat02_max_position_output;
SKOutputNumeric<float>* bat02_min_position_output;
SKOutputNumeric<float>* bat02_vdelta_output;
SKOutputNumeric<float>* bat02_pack_voltage_output;
SKOutputNumeric<float>* bat02_temp_mos_output;
SKOutputNumeric<float>* bat02_temp_max_output;
SKOutputNumeric<float>* bat02_temp_min_output;
SKOutputNumeric<float>* bat02_current_output;
SKOutputNumeric<float>* bat02_soc_output;
SKOutputNumeric<float>* bat02_soh_output;
SKOutputNumeric<float>* bat02_resma_output;
SKOutputNumeric<float>* bat02_fullma_output;
SKOutputNumeric<float>* bat02_facma_output;
SKOutputNumeric<float>* bat02_cycletime_output;
SKOutputNumeric<float>* bat02_faultfirst_output;
SKOutputNumeric<float>* bat02_faultsecond_output;
SKOutputNumeric<float>* bat02_faultthird_output;
SKOutputNumeric<float>* bat02_heatcoolerror_output;

void setupBms() {
  // -------------------------------------------------------------
  // MOD BUS for 2x BMS
  // -------------------------------------------------------------

  // Create Signal K outputs for Battery 01 (addr 0x01)
  // Create metadata and SKOutputNumeric for 32 cells
  bat01_cell_outputs.reserve(32);
  bat02_cell_outputs.reserve(32);
  for (int i = 1; i <= 32; i++) {
    auto* cell_metadata = new signalk_extended_metadata("V", "Cell " + String(i), "Battery 01 cell voltage");
    cell_metadata->add_zone(0.0f, 2.5f, String("fault"), String("Cell voltage critically low"));
    cell_metadata->add_zone(2.5f, 2.9f, String("warn"), String("Cell voltage low"));
    cell_metadata->add_zone(-1, 0.0, String("nominal"), String("Nominal/normal range not defined"));
    cell_metadata->add_zone(3.55f, 3.65f, String("warn"), String("Cell voltage high"));
    cell_metadata->add_zone(3.65f, NAN, String("fault"), String("Cell voltage critically high"));
    auto* cell_output = new SKOutputNumeric<float>("electrical.batteries.bat01.cells.cell" + String(i), "/bat01_cell" + String(i), cell_metadata);
    bat01_cell_outputs.push_back(cell_output);
    bat02_cell_outputs.push_back(cell_output);
  }
  // --- Battery 01 outputs (struct order) ---
  // wBnAFE1[16]
  bat01_bnAFE1_outputs.reserve(16);
  for (int i = 1; i <= 16; i++) {
    auto* meta = new signalk_extended_metadata("", "BnAFE1 " + String(i), "Battery 01 balancing state AFE1");
    auto* output = new SKOutputNumeric<int>("electrical.batteries.bat01.cells.bnAFE1." + String(i), "/bat01_bnAFE1_" + String(i), meta);
    bat01_bnAFE1_outputs.push_back(output);
  }
  // wBnAFE2[16]
  bat01_bnAFE2_outputs.reserve(16);
  for (int i = 1; i <= 16; i++) {
    auto* meta = new signalk_extended_metadata("", "BnAFE2 " + String(i), "Battery 01 balancing state AFE2");
    auto* output = new SKOutputNumeric<int>("electrical.batteries.bat01.cells.bnAFE2." + String(i), "/bat01_bnAFE2_" + String(i), meta);
    bat01_bnAFE2_outputs.push_back(output);
  }
  // VcellStatistics
  auto* bat01_max_cell_metadata = new signalk_extended_metadata("V", "Max Cell Voltage", "Battery 01 max cell voltage");
  bat01_max_cell_metadata->add_zone(-1, 3.55f, "warn", "Max cell voltage high");
  bat01_max_cell_metadata->add_zone(-1, 3.65f, "fault", "Max cell voltage critically high");
  bat01_max_cell_output = new SKOutputNumeric<float>("electrical.batteries.bat01.cells.maxVoltage", "/bat01_max_cell_voltage", bat01_max_cell_metadata);

  auto* bat01_min_cell_metadata = new signalk_extended_metadata("V", "Min Cell Voltage", "Battery 01 min cell voltage");
  bat01_min_cell_metadata->add_zone(2.9f, -1, "warn", "Min cell voltage low");
  bat01_min_cell_metadata->add_zone(2.6f, -1, "fault", "Min cell voltage critically low");
  bat01_min_cell_output = new SKOutputNumeric<float>("electrical.batteries.bat01.cells.minVoltage", "/bat01_min_cell_voltage", bat01_min_cell_metadata);

  bat01_max_position_output = new SKOutputNumeric<float>("electrical.batteries.bat01.cells.maxPosition", "/bat01_max_position", nullptr);
  bat01_min_position_output = new SKOutputNumeric<float>("electrical.batteries.bat01.cells.minPosition", "/bat01_min_position", nullptr);
  bat01_vdelta_output = new SKOutputNumeric<float>("electrical.batteries.bat01.cells.vdelta", "/bat01_vdelta", nullptr);

  auto* bat01_pack_voltage_metadata = new signalk_extended_metadata("V", "Pack Voltage", "Battery 01 pack voltage");
  bat01_pack_voltage_metadata->add_zone(0.0f, 45.0f, "alarm", "Pack voltage critically low");
  bat01_pack_voltage_metadata->add_zone(45.0f, 48.0f, "warn", "Pack voltage low");
  bat01_pack_voltage_metadata->add_zone(48.0f, 55.0f, "normal", "Normal pack voltage");
  bat01_pack_voltage_metadata->add_zone(55.0f, 60.0f, "alarm", "Pack voltage critically high");
  bat01_pack_voltage_output = new SKOutputNumeric<float>("electrical.batteries.bat01.voltage", "/bat01_pack_voltage", bat01_pack_voltage_metadata);

  // wTemp[6]
  bat01_temp_outputs.reserve(6);
  for (int t = 1; t <= 6; t++) {
    auto* meta = new signalk_extended_metadata("K", "Cell Temp " + String(t), "Battery 01 cell temperature " + String(t));
    meta->add_zone(-1, 0.0f, "nominal", "Nominal/Invalid temperature");
    meta->add_zone(0, 313.15f, "normal", "Normal temperature");
    meta->add_zone(313.15f, 343.15f, "warn", "Approaching thermal limit");
    meta->add_zone(343.15f, 363.15f, "alarm", "Thermal alarm - reduce charge/discharge");
    meta->add_zone(363.15f, NAN, "emergency", "Critical temperature - shutdown required");
    auto* output = new SKOutputNumeric<float>("electrical.batteries.bat01.temperature.cell" + String(t), "/bat01_temp_cell" + String(t), meta);
    bat01_temp_outputs.push_back(output);
  }
  // wTempEnv[3]
  bat01_temp_env_outputs.reserve(3);
  for (int t = 1; t <= 3; t++) {
    auto* meta = new signalk_extended_metadata("K", "Env Temp " + String(t), "Battery 01 environment temperature " + String(t));
    meta->add_zone(-1, 0.0f, "nominal", "Nominal/Invalid temperature");
    meta->add_zone(0, 313.15f, "normal", "Normal temperature");
    meta->add_zone(313.15f, 343.15f, "warn", "Approaching thermal limit");
    meta->add_zone(343.15f, 363.15f, "alarm", "Thermal alarm - reduce charge/discharge");
    meta->add_zone(363.15f, NAN, "emergency", "Critical temperature - shutdown required");
    auto* output = new SKOutputNumeric<float>("electrical.batteries.bat01.temperature.env" + String(t), "/bat01_temp_env" + String(t), meta);
    bat01_temp_env_outputs.push_back(output);
  }
  // wTempMOS
  auto* bat01_temp_mos_metadata = new signalk_extended_metadata("K", "MOS Temp", "Battery 01 MOS temperature");
  bat01_temp_mos_metadata->add_zone(-1, 0.0f, "nominal", "Nominal/Invalid temperature");
  bat01_temp_mos_metadata->add_zone(0, 313.15f, "normal", "Normal temperature");
  bat01_temp_mos_metadata->add_zone(313.15f, 343.15f, "warn", "Approaching thermal limit");
  bat01_temp_mos_metadata->add_zone(343.15f, 363.15f, "alarm", "Thermal alarm - reduce charge/discharge");
  bat01_temp_mos_metadata->add_zone(363.15f, NAN, "emergency", "Critical temperature - shutdown required");
  bat01_temp_mos_output = new SKOutputNumeric<float>("electrical.batteries.bat01.temperature.mos", "/bat01_temp_mos", bat01_temp_mos_metadata);
  // wTempMax
  auto* bat01_temp_max_metadata = new signalk_extended_metadata("K", "Max Temp", "Battery 01 max temperature");
  bat01_temp_max_metadata->add_zone(-1, 0.0f, "nominal", "Nominal/Invalid temperature");
  bat01_temp_max_metadata->add_zone(0, 313.15f, "normal", "Normal temperature");
  bat01_temp_max_metadata->add_zone(313.15f, 343.15f, "warn", "Approaching thermal limit");
  bat01_temp_max_metadata->add_zone(343.15f, 363.15f, "alarm", "Thermal alarm - reduce charge/discharge");
  bat01_temp_max_metadata->add_zone(363.15f, NAN, "emergency", "Critical temperature - shutdown required");
  bat01_temp_max_output = new SKOutputNumeric<float>("electrical.batteries.bat01.temperature.max", "/bat01_temp_max", bat01_temp_max_metadata);
  // wTempMin
  auto* bat01_temp_min_metadata = new signalk_extended_metadata("K", "Min Temp", "Battery 01 min temperature");
  bat01_temp_min_metadata->add_zone(-1, 0.0f, "nominal", "Nominal/Invalid temperature");
  bat01_temp_min_metadata->add_zone(0, 313.15f, "normal", "Normal temperature");
  bat01_temp_min_metadata->add_zone(313.15f, 343.15f, "warn", "Approaching thermal limit");
  bat01_temp_min_metadata->add_zone(343.15f, 363.15f, "alarm", "Thermal alarm - reduce charge/discharge");
  bat01_temp_min_metadata->add_zone(363.15f, NAN, "emergency", "Critical temperature - shutdown required");
  bat01_temp_min_output = new SKOutputNumeric<float>("electrical.batteries.bat01.temperature.min", "/bat01_temp_min", bat01_temp_min_metadata);

  // Current/SOC/SOH/other fields
  auto* bat01_current_metadata = new signalk_extended_metadata("A", "Current", "Battery 01 current");
  bat01_current_metadata->add_zone(-200.0f, -150.0f, "alarm", "Excessive discharge current");
  bat01_current_metadata->add_zone(-150.0f, -80.0f, "warn", "High discharge current");
  bat01_current_metadata->add_zone(-80.0f, 80.0f, "normal", "Normal current");
  bat01_current_metadata->add_zone(80.0f, 150.0f, "warn", "High charge current");
  bat01_current_metadata->add_zone(150.0f, 200.0f, "alarm", "Excessive charge current");
  bat01_current_output = new SKOutputNumeric<float>("electrical.batteries.bat01.current", "/bat01_current", bat01_current_metadata);

  auto* bat01_soc_metadata = new signalk_extended_metadata("ratio", "SOC", "Battery 01 state of charge");
  bat01_soc_metadata->add_zone(0.0f, 0.1f, "alarm", "Battery critically discharged");
  bat01_soc_metadata->add_zone(0.1f, 0.2f, "warn", "Battery low");
  bat01_soc_metadata->add_zone(0.2f, 0.8f, "normal", "Normal state of charge");
  bat01_soc_metadata->add_zone(0.8f, 1.0f, "warn", "Battery nearly full");
  bat01_soc_output = new SKOutputNumeric<float>("electrical.batteries.bat01.capacity.stateOfCharge", "/bat01_soc", bat01_soc_metadata);

  auto* bat01_soh_metadata = new signalk_extended_metadata("ratio", "SOH", "Battery 01 state of health");
  bat01_soh_output = new SKOutputNumeric<float>("electrical.batteries.bat01.capacity.stateOfHealth", "/bat01_soh", bat01_soh_metadata);

  bat01_resma_output = new SKOutputNumeric<float>("electrical.batteries.bat01.current.resmA", "/bat01_resma", bat01_current_metadata);
  bat01_fullma_output = new SKOutputNumeric<float>("electrical.batteries.bat01.current.fullmA", "/bat01_fullma", bat01_current_metadata);
  bat01_facma_output = new SKOutputNumeric<float>("electrical.batteries.bat01.current.facmA", "/bat01_facma", bat01_current_metadata);
  // THESE NEED METADATA THAT IS RELEVNAT TO THEIR MEANING - CURRENTLY USING THE SAME AS CURRENT, BUT THIS IS NOT
  // ACCURATE OR HELPFUL FOR END USERS - NEED TO DEFINE ZONES AND WARN/ALARM LEVELS BASED ON THEIR ACTUAL MEANING
  bat01_cycletime_output = new SKOutputNumeric<float>("electrical.batteries.bat01.current.cycleTime", "/bat01_cycletime", bat01_current_metadata);
  bat01_faultfirst_output = new SKOutputNumeric<float>("electrical.batteries.bat01.fault.first", "/bat01_faultfirst", bat01_current_metadata);
  bat01_faultsecond_output = new SKOutputNumeric<float>("electrical.batteries.bat01.fault.second", "/bat01_faultsecond", bat01_current_metadata);
  bat01_faultthird_output = new SKOutputNumeric<float>("electrical.batteries.bat01.fault.third", "/bat01_faultthird", bat01_current_metadata);
  bat01_heatcoolerror_output = new SKOutputNumeric<float>("electrical.batteries.bat01.heatCoolError", "/bat01_heatcoolerror", bat01_current_metadata);
  
    // --- Battery 02 outputs (struct order) ---
    // wBnAFE1[16]
     bat02_bnAFE1_outputs.reserve(16);
    for (int i = 1; i <= 16; i++) {
      auto* meta = new signalk_extended_metadata("", "BnAFE1 " + String(i), "Battery 02 balancing state AFE1");
      auto* output = new SKOutputNumeric<int>("electrical.batteries.bat02.cells.bnAFE1." + String(i), "/bat02_bnAFE1_" + String(i), meta);
      bat02_bnAFE1_outputs.push_back(output);
    }
    // wBnAFE2[16]
     bat02_bnAFE2_outputs.reserve(16);
    for (int i = 1; i <= 16; i++) {
      auto* meta = new signalk_extended_metadata("", "BnAFE2 " + String(i), "Battery 02 balancing state AFE2");
      auto* output = new SKOutputNumeric<int>("electrical.batteries.bat02.cells.bnAFE2." + String(i), "/bat02_bnAFE2_" + String(i), meta);
      bat02_bnAFE2_outputs.push_back(output);
    }
    // VcellStatistics
    auto* bat02_max_cell_metadata = new signalk_extended_metadata("V", "Max Cell Voltage", "Battery 02 max cell voltage");
    bat02_max_cell_metadata->add_zone(-1, 3.5f, "warn", "Max cell voltage high");
    bat02_max_cell_metadata->add_zone(-1, 3.8f, "fault", "Max cell voltage critically high");
    bat02_max_cell_output = new SKOutputNumeric<float>("electrical.batteries.bat02.cells.maxVoltage", "/bat02_max_cell_voltage", bat02_max_cell_metadata);

    auto* bat02_min_cell_metadata = new signalk_extended_metadata("V", "Min Cell Voltage", "Battery 02 min cell voltage");
    bat02_min_cell_metadata->add_zone(3.0f, -1, "warn", "Min cell voltage low");
    bat02_min_cell_metadata->add_zone(2.8f, -1, "fault", "Min cell voltage critically low");
    bat02_min_cell_output = new SKOutputNumeric<float>("electrical.batteries.bat02.cells.minVoltage", "/bat02_min_cell_voltage", bat02_min_cell_metadata);

    bat02_max_position_output = new SKOutputNumeric<float>("electrical.batteries.bat02.cells.maxPosition", "/bat02_max_position", nullptr);
    bat02_min_position_output = new SKOutputNumeric<float>("electrical.batteries.bat02.cells.minPosition", "/bat02_min_position", nullptr);
    bat02_vdelta_output = new SKOutputNumeric<float>("electrical.batteries.bat02.cells.vdelta", "/bat02_vdelta", nullptr);

    auto* bat02_pack_voltage_metadata = new signalk_extended_metadata("V", "Pack Voltage", "Battery 02 pack voltage");
    bat02_pack_voltage_metadata->add_zone(0.0f, 45.0f, "alarm", "Pack voltage critically low");
    bat02_pack_voltage_metadata->add_zone(45.0f, 48.0f, "warn", "Pack voltage low");
    bat02_pack_voltage_metadata->add_zone(48.0f, 55.0f, "normal", "Normal pack voltage");
    bat02_pack_voltage_metadata->add_zone(55.0f, 60.0f, "alarm", "Pack voltage critically high");
    bat02_pack_voltage_output = new SKOutputNumeric<float>("electrical.batteries.bat02.voltage", "/bat02_pack_voltage", bat02_pack_voltage_metadata);

    // wTemp[6]
    bat02_temp_outputs.reserve(6);
    for (int t = 1; t <= 6; t++) {
      auto* meta = new signalk_extended_metadata("K", "Cell Temp " + String(t), "Battery 02 cell temperature " + String(t));
      meta->add_zone(-1, 0.0f, "nominal", "Nominal/Invalid temperature");
      meta->add_zone(0, 313.15f, "normal", "Normal temperature");
      meta->add_zone(313.15f, 343.15f, "warn", "Approaching thermal limit");
      meta->add_zone(343.15f, 363.15f, "alarm", "Thermal alarm - reduce charge/discharge");
      meta->add_zone(363.15f, NAN, "emergency", "Critical temperature - shutdown required");
      auto* output = new SKOutputNumeric<float>("electrical.batteries.bat02.temperature.cell" + String(t), "/bat02_temp_cell" + String(t), meta);
      bat02_temp_outputs.push_back(output);
    }
    // wTempEnv[3]
     bat02_temp_env_outputs.reserve(3);
    for (int t = 1; t <= 3; t++) {
      auto* meta = new signalk_extended_metadata("K", "Env Temp " + String(t), "Battery 02 environment temperature " + String(t));
      meta->add_zone(-1, 0.0f, "nominal", "Nominal/Invalid temperature");
      meta->add_zone(0, 313.15f, "normal", "Normal temperature");
      meta->add_zone(313.15f, 343.15f, "warn", "Approaching thermal limit");
      meta->add_zone(343.15f, 363.15f, "alarm", "Thermal alarm - reduce charge/discharge");
      meta->add_zone(363.15f, NAN, "emergency", "Critical temperature - shutdown required");
      auto* output = new SKOutputNumeric<float>("electrical.batteries.bat02.temperature.env" + String(t), "/bat02_temp_env" + String(t), meta);
      bat02_temp_env_outputs.push_back(output);
    }
    // wTempMOS
    auto* bat02_temp_mos_metadata = new signalk_extended_metadata("K", "MOS Temp", "Battery 02 MOS temperature");
    bat02_temp_mos_metadata->add_zone(-1, 0.0f, "nominal", "Nominal/Invalid temperature");
    bat02_temp_mos_metadata->add_zone(0, 313.15f, "normal", "Normal temperature");
    bat02_temp_mos_metadata->add_zone(313.15f, 343.15f, "warn", "Approaching thermal limit");
    bat02_temp_mos_metadata->add_zone(343.15f, 363.15f, "alarm", "Thermal alarm - reduce charge/discharge");
    bat02_temp_mos_metadata->add_zone(363.15f, NAN, "emergency", "Critical temperature - shutdown required");
    bat02_temp_mos_output = new SKOutputNumeric<float>("electrical.batteries.bat02.temperature.mos", "/bat02_temp_mos", bat02_temp_mos_metadata);
    // wTempMax
    auto* bat02_temp_max_metadata = new signalk_extended_metadata("K", "Max Temp", "Battery 02 max temperature");
    bat02_temp_max_metadata->add_zone(-1, 0.0f, "nominal", "Nominal/Invalid temperature");
    bat02_temp_max_metadata->add_zone(0, 313.15f, "normal", "Normal temperature");
    bat02_temp_max_metadata->add_zone(313.15f, 343.15f, "warn", "Approaching thermal limit");
    bat02_temp_max_metadata->add_zone(343.15f, 363.15f, "alarm", "Thermal alarm - reduce charge/discharge");
    bat02_temp_max_metadata->add_zone(363.15f, NAN, "emergency", "Critical temperature - shutdown required");
    bat02_temp_max_output = new SKOutputNumeric<float>("electrical.batteries.bat02.temperature.max", "/bat02_temp_max", bat02_temp_max_metadata);
    // wTempMin
    auto* bat02_temp_min_metadata = new signalk_extended_metadata("K", "Min Temp", "Battery 02 min temperature");
    bat02_temp_min_metadata->add_zone(-1, 0.0f, "nominal", "Nominal/Invalid temperature");
    bat02_temp_min_metadata->add_zone(0, 313.15f, "normal", "Normal temperature");
    bat02_temp_min_metadata->add_zone(313.15f, 343.15f, "warn", "Approaching thermal limit");
    bat02_temp_min_metadata->add_zone(343.15f, 363.15f, "alarm", "Thermal alarm - reduce charge/discharge");
    bat02_temp_min_metadata->add_zone(363.15f, NAN, "emergency", "Critical temperature - shutdown required");
    bat02_temp_min_output = new SKOutputNumeric<float>("electrical.batteries.bat02.temperature.min", "/bat02_temp_min", bat02_temp_min_metadata);

    // Current/SOC/SOH/other fields
    auto* bat02_current_metadata = new signalk_extended_metadata("A", "Current", "Battery 02 current");
    bat02_current_metadata->add_zone(-200.0f, -150.0f, "alarm", "Excessive discharge current");
    bat02_current_metadata->add_zone(-150.0f, -80.0f, "warn", "High discharge current");
    bat02_current_metadata->add_zone(-80.0f, 80.0f, "normal", "Normal current");
    bat02_current_metadata->add_zone(80.0f, 150.0f, "warn", "High charge current");
    bat02_current_metadata->add_zone(150.0f, 200.0f, "alarm", "Excessive charge current");
    bat02_current_output = new SKOutputNumeric<float>("electrical.batteries.bat02.current", "/bat02_current", bat02_current_metadata);
    auto* bat02_soc_metadata = new signalk_extended_metadata("ratio", "SOC", "Battery 02 state of charge");
    bat02_soc_metadata->add_zone(0.0f, 0.1f, "alarm", "Battery critically discharged");
    bat02_soc_metadata->add_zone(0.1f, 0.2f, "warn", "Battery low");
    bat02_soc_metadata->add_zone(0.2f, 0.8f, "normal", "Normal state of charge");
    bat02_soc_metadata->add_zone(0.8f, 1.0f, "warn", "Battery nearly full");
    bat02_soc_output = new SKOutputNumeric<float>("electrical.batteries.bat02.capacity.stateOfCharge", "/bat02_soc", bat02_soc_metadata);
    auto* bat02_soh_metadata = new signalk_extended_metadata("ratio", "SOH", "Battery 02 state of health");
    bat02_soh_output = new SKOutputNumeric<float>("electrical.batteries.bat02.capacity.stateOfHealth", "/bat02_soh", bat02_soh_metadata);

    bat02_resma_output = new SKOutputNumeric<float>("electrical.batteries.bat02.current.resmA", "/bat02_resma", bat02_current_metadata);
    bat02_fullma_output = new SKOutputNumeric<float>("electrical.batteries.bat02.current.fullmA", "/bat02_fullma", bat02_current_metadata);
    bat02_facma_output = new SKOutputNumeric<float>("electrical.batteries.bat02.current.facmA", "/bat02_facma", bat02_current_metadata);
    // THESE NEED METADATA THAT IS RELEVNAT TO THEIR MEANING - CURRENTLY USING THE SAME AS CURRENT, BUT THIS IS NOT
    // ACCURATE OR HELPFUL FOR END USERS - NEED TO DEFINE ZONES AND WARN/ALARM LEVELS BASED ON THEIR ACTUAL MEANING
    bat02_cycletime_output = new SKOutputNumeric<float>("electrical.batteries.bat02.current.cycleTime", "/bat02_cycletime", bat02_current_metadata);
    bat02_faultfirst_output = new SKOutputNumeric<float>("electrical.batteries.bat02.fault.first", "/bat02_faultfirst", bat02_current_metadata);
    bat02_faultsecond_output = new SKOutputNumeric<float>("electrical.batteries.bat02.fault.second", "/bat02_faultsecond", bat02_current_metadata);
    bat02_faultthird_output = new SKOutputNumeric<float>("electrical.batteries.bat02.fault.third", "/bat02_faultthird", bat02_current_metadata);
    bat02_heatcoolerror_output = new SKOutputNumeric<float>("electrical.batteries.bat02.heatCoolError", "/bat02_heatcoolerror", bat02_current_metadata);
  
  BMS_modbus.onData([](uint8_t serverAddress, esp32Modbus::FunctionCode fc, uint16_t address, uint8_t* data, size_t length) {
    // This callback only fires if CRC is VALID
    // debugI("✓ DATA RECEIVED (CRC OK) - Addr=0x%04X Slave=0x%02x Length=%d",
    // address, serverAddress, length);
    switch (address) {
    case 0xD000: {
      /*         // Debug: Show raw data for D0000026
              String hex_dump = "RAW D0000 (";
              hex_dump += String(length);
              hex_dump += " bytes): ";
              for (size_t i = 0; i < length && i < 10; i++) {
                hex_dump += String(data[i], HEX);
                hex_dump += " ";
              }
              if (length > 10) hex_dump += "...";
              debugI("%s", hex_dump.c_str());
       */
      if (serverAddress == 0x01) {
        Parse_D0000026(&sBMS0_data, data, length);
        //   debugI("✓ BAT01 D0000 PARSED: Cell[0]=0x%04x Cell[1]=0x%04x
        //   VMax=0x%04x VMin=0x%04x\n",
        //          sBMS0_data.wVcellAFE1[0], sBMS0_data.wVcellAFE1[1],
        //          sBMS0_data.sVcellStatistics.wVcellMax,
        //          sBMS0_data.sVcellStatistics.wVcellMin);
      } else if (serverAddress == 0x02) {
        Parse_D0000026(&sBMS1_data, data, length);
        // debugI("✓ BAT02 D0000 PARSED: Cell[0]=0x%04x Cell[1]=0x%04x
        // VMax=0x%04x VMin=0x%04x\n",
        //        sBMS1_data.wVcellAFE1[0], sBMS1_data.wVcellAFE1[1],
        //        sBMS1_data.sVcellStatistics.wVcellMax,
        //        sBMS1_data.sVcellStatistics.wVcellMin);
      }
    } break;
    case 0xD026: {
      // Debug: Show raw data for D026
      // String hex_dump = "RAW D026 (";
      // hex_dump += String(length);
      // hex_dump += " bytes): ";
      // for (size_t i = 0; i < length && i < 50; i++) {
      //   if (data[i] < 0x10) hex_dump += "0";
      //   hex_dump += String(data[i], HEX);
      //   hex_dump += " ";
      // }
      // if (length > 50) hex_dump += "...";
      // debugI("%s", hex_dump.c_str());

      if (serverAddress == 0x01) {
        Parse_D0260019(&sBMS0_data, data, length);
        //   debugI("✓ BAT01 D026 PARSED: SOC=%d Icharge=%d Idischarge=%d
        //   TempMax=%d",
        //          sBMS0_data.sCurrentSocHeatCoolFault.wSOC,
        //          sBMS0_data.sCurrentSocHeatCoolFault.wIcharge,
        //          sBMS0_data.sCurrentSocHeatCoolFault.wIdischarge,
        //          sBMS0_data.sTemperatures.wTempMax);
      } else if (serverAddress == 0x02) {
        Parse_D0260019(&sBMS1_data, data, length);
        //   debugI("✓ BAT02 D026 PARSED: SOC=%d Icharge=%d Idischarge=%d
        //   TempMax=%d",
        //          sBMS1_data.sCurrentSocHeatCoolFault.wSOC,
        //          sBMS1_data.sCurrentSocHeatCoolFault.wIcharge,
        //          sBMS1_data.sCurrentSocHeatCoolFault.wIdischarge,
        //          sBMS1_data.sTemperatures.wTempMax);
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
      // debugI("D200 response received from slave 0x%02X, length=%d",
      // serverAddress, length);
      if (serverAddress == 0x01) {
        Parse_D2000001(&sBMS0_data, data, length);
        //   debugI("✓ BAT01 D200 PARSED: wSOC=0x%04X (%d)",
        //          sBMS0_data.sCurrentSocHeatCoolFault.wSOC,
        //          sBMS0_data.sCurrentSocHeatCoolFault.wSOC);
        // } else if (serverAddress == 0x02) {
        Parse_D2000001(&sBMS1_data, data, length);
        //   debugI("✓ BAT02 D200 PARSED: wSOC=0x%04X (%d)",
        //          sBMS1_data.sCurrentSocHeatCoolFault.wSOC,
        //          sBMS1_data.sCurrentSocHeatCoolFault.wSOC);
      }
    } break;
    }
  });
  BMS_modbus.onError([](esp32Modbus::Error error) {
    const char* error_str = "UNKNOWN";
    switch (error) {
    case esp32Modbus::SUCCES:
      error_str = "SUCCESS";
      break;
    case esp32Modbus::ILLEGAL_FUNCTION:
      error_str = "ILLEGAL_FUNCTION";
      break;
    case esp32Modbus::ILLEGAL_DATA_ADDRESS:
      error_str = "ILLEGAL_DATA_ADDRESS";
      break;
    case esp32Modbus::ILLEGAL_DATA_VALUE:
      error_str = "ILLEGAL_DATA_VALUE";
      break;
    case esp32Modbus::SERVER_DEVICE_FAILURE:
      error_str = "SERVER_DEVICE_FAILURE";
      break;
    case esp32Modbus::ACKNOWLEDGE:
      error_str = "ACKNOWLEDGE";
      break;
    case esp32Modbus::SERVER_DEVICE_BUSY:
      error_str = "SERVER_DEVICE_BUSY";
      break;
    case esp32Modbus::NEGATIVE_ACKNOWLEDGE:
      error_str = "NEGATIVE_ACKNOWLEDGE";
      break;
    case esp32Modbus::MEMORY_PARITY_ERROR:
      error_str = "MEMORY_PARITY_ERROR";
      break;
    case esp32Modbus::TIMEOUT:
      error_str = "TIMEOUT";
      break;
    case esp32Modbus::INVALID_SLAVE:
      error_str = "INVALID_SLAVE";
      break;
    case esp32Modbus::INVALID_FUNCTION:
      error_str = "INVALID_FUNCTION";
      break;
    case esp32Modbus::CRC_ERROR: {
      error_str = "CRC_ERROR";
      debugW("!!! CRC ERROR on response to: Slave=0x%02x Address=0x%04x", last_requested_slave, last_requested_address);
      debugW("    Device DID respond (we have data), but CRC validation "
             "failed.");
      debugW("    This usually means: CRC-16 variant mismatch between library "
             "and device");
      break;
    }
    case esp32Modbus::COMM_ERROR:
      error_str = "COMM_ERROR";
      break;
    default:
      break;
    }
    debugW("⚠ MODBUS ERROR: 0x%02x (%s)", static_cast<uint8_t>(error), error_str);
  });

  debugD("Initializing Serial1 at 19200 baud, RX=pin 18, TX=pin 17");
  Serial1.begin(19200, SERIAL_8N1, ModBusRxdPin,
                ModBusTxdPin); // Modbus connection
  debugD("Calling BMS_modbus.begin()");
  BMS_modbus.begin();
  debugD("BMS_modbus initialized successfully");

  // Increase timeout for large responses (D0000 is 76 bytes @ 19200 baud =
  // ~40ms + buffer)
  BMS_modbus.setTimeOutValue(500); // 500ms timeout should be plenty
  debugD("BMS_modbus timeout set to 500ms");

  // Periodic update of battery data to Signal K
  app.onRepeat(1000, []() {
    // --------------------------------------------------------------------------
    // Battery 01 - Forward battery
    // --------------------------------------------------------------------------
    for (int i = 0; i < 16; i++) {
      bat01_cell_outputs[i]->set(sBMS0_data.wVcellAFE1[i]);
    }
    for (int i = 0; i < 16; i++) {
      bat01_cell_outputs[16 + i]->set(sBMS0_data.wVcellAFE2[i]);
    }
    bat01_max_cell_output->set(sBMS0_data.sVcellStatistics.wVcellMax);
    bat01_max_position_output->set(sBMS0_data.sVcellStatistics.wMaxPosition);
    bat01_min_position_output->set(sBMS0_data.sVcellStatistics.wMinPosition);
    bat01_vdelta_output->set(sBMS0_data.sVcellStatistics.wVdelta);

    bat01_pack_voltage_output->set(sBMS0_data.sVcellStatistics.wVbat);
    //     sDisplayData.ForwardBattery.Voltage = sBMS0_data.sVcellStatistics.wVbat / 10; // Display in 100mV (512
    //     = 51.2V) 
    //sDisplayData.ForwardBattery.BmsCommsOk = true;
    //   }
    //   // Current: positive for charging, negative for discharging
    //   float bat01_current = (float)sBMS0_data.sCurrentSocHeatCoolFault.wIcharge / 1000.0f; // Convert mA to A
    //   if (sBMS0_data.sCurrentSocHeatCoolFault.wIdischarge > 0) {
    //     bat01_current = -(float)sBMS0_data.sCurrentSocHeatCoolFault.wIdischarge / 1000.0f; // Discharge is negative
    //   }
    bat01_current_output->set(sBMS0_data.sCurrentSocHeatCoolFault.wIdischarge);
    //   sDisplayData.ForwardBattery.Current = (int32_t)(bat01_current * 10); // Display in 100mA units
    //   // SOC: convert from percentage (0-100) to ratio (0.0-1.0)
    //        debugI("BAT01 SOC raw=%d%% → %.2f",
    //              sBMS0_data.sCurrentSocHeatCoolFault.wSOC,
    //              sBMS0_data.sCurrentSocHeatCoolFault.wSOC / 100.0f);
    //   /
    bat01_soc_output->set(sBMS0_data.sCurrentSocHeatCoolFault.wSOC);
    //   sDisplayData.ForwardBattery.SoC = sBMS0_data.sCurrentSocHeatCoolFault.wSOC; // Display as percentage
    //   // Temperature: convert from 0.1°C units with +40°C offset to Kelvin
    //   if (sBMS0_data.sTemperatures.wTempMax > 0) {
    //     float temp_celsius = (sBMS0_data.sTemperatures.wTempMax / 10.0f) - 40.0f; // Remove +40 offset
    bat01_temp_max_output->set(sBMS0_data.sTemperatures.wTempMax);
    //     sDisplayData.ForwardBattery.Temp = (sBMS0_data.sTemperatures.wTempMax / 10) - 40; // Display in °C
    //   }

    // --------------------------------------------------------------------------
    // Battery 02 - Aft battery
    // --------------------------------------------------------------------------
    for (int i = 0; i < 16; i++) {
         bat02_cell_outputs[i]->set(sBMS1_data.wVcellAFE1[i]);
    }
    for (int i = 0; i < 16; i++) {
        bat02_cell_outputs[16 + i]->set(sBMS1_data.wVcellAFE2[i]);
    }
    
    bat02_max_position_output->set(sBMS1_data.sVcellStatistics.wMaxPosition);
    bat02_min_position_output->set(sBMS1_data.sVcellStatistics.wMinPosition);
    bat02_vdelta_output->set(sBMS1_data.sVcellStatistics.wVdelta);

    bat02_pack_voltage_output->set(sBMS1_data.sVcellStatistics.wVbat);
    //     sDisplayData.AftBattery.Voltage = sBMS1_data.sVcellStatistics.wVbat / 10; // Display in 100mV
    //     sDisplayData.AftBattery.BmsCommsOk = true;
    //   }
    //   // Current: positive for charging, negative for discharging
    //   float bat02_current = (float)sBMS1_data.sCurrentSocHeatCoolFault.wIcharge / 1000.0f; // Convert mA to A
    //   if (sBMS1_data.sCurrentSocHeatCoolFault.wIdischarge > 0) {
    //     bat02_current = -(float)sBMS1_data.sCurrentSocHeatCoolFault.wIdischarge / 1000.0f; // Discharge is negative
    //   }
    bat02_current_output->set(sBMS1_data.sCurrentSocHeatCoolFault.wIdischarge);
    //   sDisplayData.AftBattery.Current = (int32_t)(bat02_current * 10); // Display in 100mA units
    //   // SOC: convert from percentage (0-100) to ratio (0.0-1.0)
    bat02_soc_output->set(sBMS1_data.sCurrentSocHeatCoolFault.wSOC);
    //   sDisplayData.AftBattery.SoC = sBMS1_data.sCurrentSocHeatCoolFault.wSOC; // Display as percentage
    //   // Temperature: convert from 0.1°C units with +40°C offset to Kelvin
    //   if (sBMS1_data.sTemperatures.wTempMax > 0) {
    //     float temp_celsius = (sBMS1_data.sTemperatures.wTempMax / 10.0f) - 40.0f; // Remove +40 offset
    bat02_temp_max_output->set(sBMS1_data.sTemperatures.wTempMax);
    //     sDisplayData.AftBattery.Temp = (sBMS1_data.sTemperatures.wTempMax / 10) - 40; // Display in °C
    //   }
    // });
  });

  app.onRepeat(1000, []() {
    // Poll both batteries (0x01 and 0x02) in sequence
    // state_index 0-3: Battery 01, state_index 4-7: Battery 02
    uint8_t slave = (state_index < 4) ? 0x01 : 0x02;
    uint8_t reg_index = state_index % 4;

    switch (reg_index) {
    case 0: {
      last_requested_slave = slave;
      last_requested_address = 0xD000;
      BMS_modbus.readHoldingRegisters(slave, 0xD000, 0x0026);
      break;
    }
    case 1: {
      last_requested_slave = slave;
      last_requested_address = 0xD026;
      BMS_modbus.readHoldingRegisters(slave, 0xD026, 0x0019);
      break;
    }
    case 2: {
      last_requested_slave = slave;
      last_requested_address = 0xD100;
      BMS_modbus.readHoldingRegisters(slave, 0xD100, 0x0015);
      break;
    }
    case 3: {
      last_requested_slave = slave;
      last_requested_address = 0xD200;
      BMS_modbus.readHoldingRegisters(slave, 0xD200, 0x0001);
      break;
    }
    }
    state_index = (state_index + 1) % 8; // Cycle through 0-7
  });
}
