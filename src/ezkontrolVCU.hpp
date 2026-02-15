#ifndef EZ_KONTROL_VCU
#define EZ_KONTROL_VCU

#include "ESP32-TWAI-CAN.hpp"
#include "sensesp.h"
typedef enum {
  STATE_CLOSED = 0,
  STATE_CONNECTED = 1,
  STATE_SYNCED = 2,
  STATE_MSG1 = 3,
  STATE_MSG2 = 4,
  STATE_LAST = 5
} eCanFsmState_t;
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

class ezkontrolVCU {
 private:
  uint8_t mcuID = 0;
  uint8_t vcuID = 208; //0xD0
  sMcuData_t sMcuData;
  eCanFsmState_t sCurrentCanState = STATE_CLOSED;
  uint32_t CanRxMsg = 0;
  uint32_t CanTxMsg = 0;
  uint16_t LifeSignal = 0;
  TwaiCAN* myESP32CAN;
  protected:
 public:
  ezkontrolVCU();
  ~ezkontrolVCU() {}
  bool begin(TwaiCAN* _myESP32CAN,uint8_t _mcuID);
  uint8_t getID(void){return mcuID;};
  bool checkFrame(CanFrame _RxFrame);
  bool SendSyncReply(uint8_t _mcuID);
  int16_t getThrottlePosition(void) {return sMcuData.ThrottlePosition;};
  int16_t getSpeedRpm(void) {return sMcuData.SpeedRpm;};
  int16_t getMotorTemp(void) {return sMcuData.MotorTemp;};
  int16_t getControllerTemp(void) {return sMcuData.ControllerTemp;};
  int16_t getBusVoltage(void) {return sMcuData.BusVoltage;};
  int16_t getBusCurrent(void) {return sMcuData.BusCurrent;};
  int16_t getPhaseCurrent(void) {return sMcuData.PhaseCurrent;};
  bool SendCommand(int16_t TargetPhaseCurrent, int16_t TargetSpeed,
                   int8_t ControlMode);
  static uint8_t GetSA(CanFrame aCanFrame);
  static uint8_t GetPS(CanFrame aCanFrame);
  static uint8_t GetPF(CanFrame aCanFrame);
char State[6][8] = {"CLOSED\0", "OPEN\0",   "SYNCED\0",
                    "MSG1  \0", "MSG2  \0", "LAST  \0"};

};

#endif  // EZ_KONTROL_VCU
