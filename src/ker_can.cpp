#include "ker_can.hpp"

ker_can::ker_can(uint8_t _mcu_port, uint8_t _mcu_starboard) {
  mcuPort = _mcu_port;
  mcuStarboard = _mcu_starboard;
    Serial.println("CAN bus constructor!");
  //McuPort.begin(&ESP32Can, mcuPort);
 // McuStarboard.begin(&ESP32Can, mcuStarboard);
}
ker_can::~ker_can() {}

bool ker_can::begin(uint8_t _txPin, uint8_t _rxPin, uint8_t _clkPin,
                    uint8_t _busoffPin) {
  bool ret = false;
  McuPort.begin(&ESP32Can, mcuPort);
  McuStarboard.begin(&ESP32Can, mcuStarboard);

  debugD("CAN Init Status %d\n", ESP32Can.getInit());
  ESP32Can.setPins(_txPin, _rxPin, _clkPin, _busoffPin);
  ESP32Can.setRxQueueSize(8);
  ESP32Can.setTxQueueSize(8);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

  //if (ESP32Can.begin(ESP32Can.convertSpeed(250),_txPin,_rxPin,-1,-1,8,8,NULL,NULL,NULL)) {
  if (ESP32Can.begin()) {
    debugD("CAN bus started!");
    ret = true;
  } else {
    debugD("CAN bus failed!");
  }
  return ret;
}
bool ker_can::checkReceiver(void) {
  bool ret = false;
  if (ESP32Can.readFrame(&rxFrame, 1000)) {
    debugD("Received frame: %08X  \r\n", rxFrame.identifier);
    debugD("SA = %02X, DA = %02X, PF = %02X\r\n", ezkontrolVCU::GetSA(rxFrame),ezkontrolVCU::GetPS(rxFrame),ezkontrolVCU::GetPF(rxFrame));
    debugD("Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n", 
           rxFrame.data[0], rxFrame.data[1], rxFrame.data[2], rxFrame.data[3],
           rxFrame.data[4], rxFrame.data[5], rxFrame.data[6], rxFrame.data[7]);
    debugD("McuPort=%02X, McuStarboard=%02X\n",McuPort.getID(),McuStarboard.getID());
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
