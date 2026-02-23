#ifndef _bms_parser_H_
#define _bms_parser_H_

#include <map>

// maximum is 82, but let's give it a bit of margin.
constexpr int kBMSInputBufferLength = 164;

typedef struct VcellStatistics
{
  float wVcellMax = -1.0f;
  float wVcellMin = -1.0f;
  float wMaxPosition = -1.0f;
  float wMinPosition = -1.0f;
  float wVdelta = -1.0f;
  float wVbat = -1.0f;
} sVcelStatistics_t;
typedef struct 
{
  float wTemp[6] = { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f };
  float wTempEnv[3] = { -1.0f, -1.0f, -1.0f };
  float wTempMOS = -1.0f;
  float wTempMax = -1.0f;
  float wTempMin = -1.0f;
} sTemperatures_t;

typedef struct 
{
  float wIcharge = -1.0f;
  float wIdischarge = -1.0f;
  float wSOC = -1.0f;
  float wSOH = -1.0f;
  float wResmA = -1.0f;
  float wFullmA = -1.0f;
  float wFacmA = -1.0f;
  float wCycleTime = -1.0f;
  float wFaultFirst = -1.0f;
  float wFaultSecond = -1.0f;
  float wFaultThird = -1.0f;
  float wHeatCoolError = -1.0f;
} sCurrentSocHeatCoolFault_t;

typedef struct 
{
  float wVcellAFE1[16] = { -1.0f };
  float wVcellAFE2[16] = { -1.0f };
  bool wBnAFE1[16] = { 0 };
  bool wBnAFE2[16] = { 0 };
  sVcelStatistics_t sVcellStatistics;
  sTemperatures_t sTemperatures;
  sCurrentSocHeatCoolFault_t sCurrentSocHeatCoolFault;
} sBMS_data_t;

inline void Parse_D0000026(sBMS_data_t* sBMS_data, uint8_t* data, size_t length)
{
  uint8_t i = 0;
  uint8_t cell = 0;
  for (cell = 0; cell < 16; cell++)
  {
    uint16_t raw = (data[i++]<<8) + data[i++];
    sBMS_data->wVcellAFE1[cell] = (raw == 0xEE49) ? -1.0f : (static_cast<float>(raw) / 1000.0f); // mV to V, -1.0f for invalid
  }
  for (cell = 0; cell < 16; cell++)
  {
    uint16_t raw = (data[i++]<<8) + data[i++];
    sBMS_data->wVcellAFE2[cell] = (raw == 0xEE49) ? -1.0f : (static_cast<float>(raw) / 1000.0f);
  }
  sBMS_data->sVcellStatistics.wVcellMax = static_cast<float>((data[i++]<<8) + data[i++]) / 1000.0f;
  sBMS_data->sVcellStatistics.wVcellMin = static_cast<float>((data[i++]<<8) + data[i++]) / 1000.0f;
  sBMS_data->sVcellStatistics.wMaxPosition = static_cast<float>((data[i++]<<8) + data[i++]);
  sBMS_data->sVcellStatistics.wMinPosition = static_cast<float>((data[i++]<<8) + data[i++]);
  sBMS_data->sVcellStatistics.wVdelta = static_cast<float>((data[i++]<<8) + data[i++]) / 1000.0f;
  sBMS_data->sVcellStatistics.wVbat = static_cast<float>((data[i++]<<8) + data[i++]) / 100.0f; // 10mV to V
}
inline void Parse_D0260019(sBMS_data_t* sBMS_data, uint8_t* data, size_t length)
{
  uint8_t i = 0;
  // Temperatures (12 registers = 24 bytes)
  for (int t = 0; t < 6; t++) {
    sBMS_data->sTemperatures.wTemp[t] = (static_cast<float>((data[i++]<<8) + data[i++]) / 10.0f) - 40.0f + 273.15f; // 0.1C, +40C offset
  }
  for (int t = 0; t < 3; t++) {
    sBMS_data->sTemperatures.wTempEnv[t] = (static_cast<float>((data[i++]<<8) + data[i++]) / 10.0f) - 40.0f + 273.15f; // 0.1C, +40C offset
  }
  sBMS_data->sTemperatures.wTempMOS = (static_cast<float>((data[i++]<<8) + data[i++]) / 10.0f) - 40.0f + 273.15f;
  sBMS_data->sTemperatures.wTempMax = (static_cast<float>((data[i++]<<8) + data[i++]) / 10.0f) - 40.0f + 273.15f;
  sBMS_data->sTemperatures.wTempMin = (static_cast<float>((data[i++]<<8) + data[i++]) / 10.0f) - 40.0f + 273.15f;
  // Current, SOC, faults (13 registers = 26 bytes) - continues from byte 24
  sBMS_data->sCurrentSocHeatCoolFault.wIcharge = static_cast<float>((data[i++]<<8) + data[i++]) / 1000.0f; // mA to A
  sBMS_data->sCurrentSocHeatCoolFault.wIdischarge = static_cast<float>((data[i++]<<8) + data[i++]) / 1000.0f;
  sBMS_data->sCurrentSocHeatCoolFault.wSOC = static_cast<float>((data[i++]<<8) + data[i++]) / 100.0f; // percent to ratio
  sBMS_data->sCurrentSocHeatCoolFault.wSOH = static_cast<float>((data[i++]<<8) + data[i++]) / 100.0f;
  sBMS_data->sCurrentSocHeatCoolFault.wResmA = static_cast<float>((data[i++]<<8) + data[i++]) / 1000.0f;
  sBMS_data->sCurrentSocHeatCoolFault.wFullmA = static_cast<float>((data[i++]<<8) + data[i++]) / 1000.0f;
  sBMS_data->sCurrentSocHeatCoolFault.wFacmA = static_cast<float>((data[i++]<<8) + data[i++]) / 1000.0f;
  sBMS_data->sCurrentSocHeatCoolFault.wCycleTime = static_cast<float>((data[i++]<<8) + data[i++]);
  sBMS_data->sCurrentSocHeatCoolFault.wFaultFirst = static_cast<float>((data[i++]<<8) + data[i++]);
  sBMS_data->sCurrentSocHeatCoolFault.wFaultSecond = static_cast<float>((data[i++]<<8) + data[i++]);
  sBMS_data->sCurrentSocHeatCoolFault.wFaultThird = static_cast<float>((data[i++]<<8) + data[i++]);
  sBMS_data->sCurrentSocHeatCoolFault.wHeatCoolError = static_cast<float>((data[i++]<<8) + data[i++]);
}

/**
 * Parse SOC data from register 0xD200 (1 register)
 * Contains State of Charge value
 */
inline void Parse_D2000001(sBMS_data_t* sBMS_data, uint8_t* data, size_t length)
{
  uint8_t i = 0;
  // If you want to parse SOC here, cast to float:
  // sBMS_data->sCurrentSocHeatCoolFault.wSOC = static_cast<float>((data[i++]<<8) + data[i++]);
}

inline void Parse_D1000015(sBMS_data_t* sBMS_data, uint8_t* data, size_t length)
{
  uint8_t i = 0;
}

#endif
