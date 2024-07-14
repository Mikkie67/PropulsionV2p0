#include "ker_can.hpp"

ker_can::ker_can(uint8_t _mcu_port, uint8_t _mcu_starboard) {
  McuPort.begin(&ESP32Can, _mcu_port);
  McuStarboard.begin(&ESP32Can, _mcu_starboard);
}
ker_can::~ker_can() {}

bool ker_can::begin(uint8_t _txPin, uint8_t _rxPin, uint8_t _clkPin,
                    uint8_t _busoffPin) {
  bool ret = false;
  Serial.printf("CAN Init Status %d", ESP32Can.getInit());
  ESP32Can.setPins(_txPin, _rxPin, _clkPin, _busoffPin);
  ESP32Can.setRxQueueSize(8);
  ESP32Can.setTxQueueSize(8);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(250));
  if (ESP32Can.begin()) {
    Serial.println("CAN bus started!");
    ret = true;
  } else {
    Serial.println("CAN bus failed!");
  }
  return ret;
}
bool ker_can::checkReceiver(void) {
  bool ret = false;
  if (ESP32Can.readFrame(&rxFrame, 1000)) {
    Serial.printf("Received frame: %08X  \r\n", rxFrame.identifier);
    Serial.printf("SA = %02X, DA = %02X, PF = %02X\r\n", ezkontrolVCU::GetSA(rxFrame),ezkontrolVCU::GetPS(rxFrame),ezkontrolVCU::GetPF(rxFrame));
    if (ezkontrolVCU::GetSA(rxFrame) == McuPort.getID()) {
      ret = McuPort.checkFrame(rxFrame);
    }
    if (ezkontrolVCU::GetSA(rxFrame) == McuStarboard.getID()) {
      ret = McuStarboard.checkFrame(rxFrame);
    }
  }
  return ret;
}
bool ker_can::SendCommands(int16_t TargetPhaseCurrent, int16_t TargetSpeed,
                           int8_t ControlMode) {
  bool ret = false;
  McuPort.SendCommand(TargetPhaseCurrent, TargetSpeed, ControlMode);
  McuStarboard.SendCommand(TargetPhaseCurrent, TargetSpeed, ControlMode);
  return ret;
}
