#ifndef _KER_CAN_
#define _KER_CAN_
#include "ESP32-TWAI-CAN.hpp"
#include "ezkontrolVCU.hpp"

class ker_can {
 private:
  TwaiCAN ESP32Can;
  CanFrame rxFrame;
  CanFrame txFrame;
  uint8_t LifeSignal = 0;
  eCanFsmState_t sCurrentCan0State = STATE_CLOSED;
  uint32_t CanRxMsg = 0;
  uint32_t CanTxMsg = 0;

 public:
  ker_can(uint8_t _mcu_port, uint8_t _mcu_starboard);
  ~ker_can();
  bool begin(uint8_t _txPin, uint8_t _rxPin, uint8_t _clkPin,
             uint8_t _busoffPin);
  bool checkReceiver(void);
  bool SendCommands(int16_t TargetPhaseCurrent, int16_t TargetSpeed,
                    int8_t ControlMode);
                    uint8_t mcuPort;
                    uint8_t mcuStarboard;
  ezkontrolVCU McuPort;
  ezkontrolVCU McuStarboard;
};

#endif  // _KER_CAN_