#ifndef _bms_parser_H_
#define _bms_parser_H_

#include <map>

// maximum is 82, but let's give it a bit of margin.
constexpr int kBMSInputBufferLength = 164;

typedef struct VcellStatistics
{
  uint16_t wVcellMax;
  uint16_t wVcellMin;
  uint16_t wMaxPosition;
  uint16_t wMinPosition;
  uint16_t wVdelta;
  uint16_t wVbat;
}sVcelStatistics_t;
typedef struct 
{
  uint16_t wTemp[6];
  uint16_t wTempEnv[3];
  uint16_t wTempMOS;
  uint16_t wTempMax;
  uint16_t wTempMin;
 }sTemperatures_t;

typedef struct 
{
  uint16_t wIcharge;
  uint16_t wIdischarge;
  uint16_t wSOC;
  uint16_t wSOH;
  uint16_t wResmA;
  uint16_t wFullmA;
  uint16_t wFacmA;
  uint16_t wCycleTime;
  uint16_t wFaultFirst;
  uint16_t wFaultSecond;
  uint16_t wFaultThird;
  uint16_t wHeatCoolError;
}sCurrentSocHeatCoolFault_t;

typedef struct 
{
  uint16_t wVcellAFE1[16];
  uint16_t wVcellAFE2[16];
  bool wBnAFE1[16];
  bool wBnAFE2[16];
  sVcelStatistics_t sVcellStatistics;
  sTemperatures_t sTemperatures;
  sCurrentSocHeatCoolFault_t sCurrentSocHeatCoolFault;
} sBMS_data_t;

void Parse_D0000026(sBMS_data_t* sBMS_data, uint8_t* data, size_t length)
{
  uint8_t i = 0;
  uint8_t cell = 0;
  for (cell = 0; cell < 16; cell++)
  {
    sBMS_data->wVcellAFE1[cell] = (data[i++]<<8) + data[i++];
  }
  for (cell = 0; cell < 16; cell++)
  {
    sBMS_data->wVcellAFE2[cell] = (data[i++]<<8) + data[i++];
  }
  sBMS_data->sVcellStatistics.wVcellMax = (data[i++]<<8) + data[i++];
  sBMS_data->sVcellStatistics.wVcellMin = (data[i++]<<8) + data[i++];
  sBMS_data->sVcellStatistics.wMaxPosition = (data[i++]<<8) + data[i++];
  sBMS_data->sVcellStatistics.wMinPosition = (data[i++]<<8) + data[i++];
  sBMS_data->sVcellStatistics.wVdelta = (data[i++]<<8) + data[i++];
  sBMS_data->sVcellStatistics.wVbat = (data[i++]<<8) + data[i++];
}
void Parse_D0260019(sBMS_data_t* sBMS_data, uint8_t* data, size_t length)
{
  uint8_t i = 0;
  // Temperatures (12 registers = 24 bytes)
  sBMS_data->sTemperatures.wTemp[0] = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTemp[1] = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTemp[2] = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTemp[3] = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTemp[4] = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTemp[5] = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTempEnv[0] = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTempEnv[1] = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTempEnv[2] = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTempMOS = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTempMax = (data[i++]<<8) + data[i++];
  sBMS_data->sTemperatures.wTempMin = (data[i++]<<8) + data[i++];
  // Current, SOC, faults (13 registers = 26 bytes) - continues from byte 24
  sBMS_data->sCurrentSocHeatCoolFault.wIcharge = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wIdischarge = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wSOC = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wSOH = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wResmA = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wFullmA = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wFacmA = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wCycleTime = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wFaultFirst = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wFaultSecond = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wFaultThird = (data[i++]<<8) + data[i++];
  sBMS_data->sCurrentSocHeatCoolFault.wHeatCoolError = (data[i++]<<8) + data[i++];
}

/**
 * Parse SOC data from register 0xD200 (1 register)
 * Contains State of Charge value
 */
void Parse_D2000001(sBMS_data_t* sBMS_data, uint8_t* data, size_t length)
{
  uint8_t i = 0;
  //sBMS_data->sCurrentSocHeatCoolFault.wSOC = (data[i++]<<8) + data[i++];
}

void Parse_D1000015(sBMS_data_t* sBMS_data, uint8_t* data, size_t length)
{
  uint8_t i = 0;
}

#endif
