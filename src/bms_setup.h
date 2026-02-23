#ifndef _BMS_SETUP_H_
#define _BMS_SETUP_H_

#include <Arduino.h>
#include <vector>
#include "bms_parser.h"
#include "esp32ModbusRTU.h"
#include "propDisplay.h"

// Compound type for battery cell data
struct BatteryCellsData {
  uint16_t cells[32];      // Individual cell voltages (0-31)
  uint16_t max_voltage;    // Highest cell voltage
  uint16_t min_voltage;    // Lowest cell voltage
  uint16_t max_index;      // Which cell has max voltage
  uint16_t min_index;      // Which cell has min voltage
  uint16_t imbalance;      // Voltage delta (max - min)
};

// BMS data structures - accessible from main
extern sBMS_data_t sBMS0_data;
extern sBMS_data_t sBMS1_data;

// Modbus instance
extern esp32ModbusRTU BMS_modbus;

// Cell metadata messages for Signal K
extern std::vector<String> cell_metadata_messages;
extern int metadata_batch_index;
extern bool was_signalk_connected;

// Function declarations
void setupBms();

void build_cell_metadata();
void build_all_bms_metadata();
bool send_cell_metadata_batch();

#endif
