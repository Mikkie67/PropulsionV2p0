#include "ezkontrolVCU.hpp"

ezkontrolVCU::ezkontrolVCU() {}

bool ezkontrolVCU::begin(TwaiCAN* myESP32CAN, uint8_t _mcuID) {
  mcuID = _mcuID;
  myESP32CAN = myESP32CAN;
  return true;
}
bool ezkontrolVCU::checkFrame(CanFrame _rxFrame) {
  bool ret = false;
  if (GetPS(_rxFrame) == vcuID) {
    Serial.printf("ezkontrolVCU: CAN frame received from %2d\n",
                  GetSA(_rxFrame));
    // Check if it is a handshake message
    if ((_rxFrame.data[0] == 0x55) && (_rxFrame.data[1] == 0x55) &&
        (_rxFrame.data[2] == 0x55) && (_rxFrame.data[3] == 0x55) &&
        (_rxFrame.data[4] == 0x55) && (_rxFrame.data[5] == 0x55) &&
        (_rxFrame.data[6] == 0x55) && (_rxFrame.data[7] == 0x55)) {
      SendSyncReply(mcuID);
      sCurrentCanState = STATE_SYNCED;
      CanRxMsg++;
      ret = true;
    }
    // data messages from MCU, populate structures
    // Message I
    if (GetPF(_rxFrame) == 0x01) {
      sCurrentCanState = STATE_MSG1;
      sMcuData.BusVoltage = (_rxFrame.data[0] << 8 + _rxFrame.data[1]) * 0.1;
      sMcuData.BusCurrent = (_rxFrame.data[2] << 8 + _rxFrame.data[3]) * 0.1;
      sMcuData.PhaseCurrent = (_rxFrame.data[4] << 8 + _rxFrame.data[5]) * 0.1;
      sMcuData.SpeedRpm = (_rxFrame.data[6] << 8 + _rxFrame.data[7]) * 0.1;
      ret = true;
    }
    // Message II
    if (GetPF(_rxFrame) == 0x02) {
      sCurrentCanState = STATE_MSG2;
      sMcuData.ControllerTemp = (_rxFrame.data[0]);
      sMcuData.MotorTemp = (_rxFrame.data[1]);
      sMcuData.ThrottlePosition = (_rxFrame.data[2]);
      sMcuData.Status = (_rxFrame.data[3]);
      sMcuData.Error =
          (_rxFrame.data[4] << 24 + _rxFrame.data[5] << 16 + _rxFrame.data[6]
                            << 8 + _rxFrame.data[7]) >>
          4;
      sMcuData.LifeSignal = (_rxFrame.data[7] & 0x0F);
      ret = true;
    }
  } else {
    // do nothing, frame not meant for this node
  }
  return ret;
}
// ----------------------------------------------------------
// CAN SYNC REPLY
// ----------------------------------------------------------
bool ezkontrolVCU::SendSyncReply(uint8_t _mcuID) {
  bool bRet = false;
  CanFrame SyncFrame = {0};
  SyncFrame.identifier = 0x0C010000 + (mcuID << 8) + vcuID;
  SyncFrame.extd = 1;
  SyncFrame.data_length_code = 8;
  SyncFrame.data[0] = 0xAA;
  SyncFrame.data[1] = 0xAA;
  SyncFrame.data[2] = 0xAA;
  SyncFrame.data[3] = 0xAA;
  SyncFrame.data[4] = 0xAA;
  SyncFrame.data[5] = 0xAA;
  SyncFrame.data[6] = 0xAA;
  SyncFrame.data[7] = 0xAA;
  bRet = myESP32CAN->writeFrame(SyncFrame);  // timeout defaults to 1 ms
  if (bRet) {
    CanTxMsg++;
  }
  return bRet;
}
bool ezkontrolVCU::SendCommand(int16_t TargetPhaseCurrent, int16_t TargetSpeed,
                               int8_t ControlMode) {
  bool ret = false;
  if (sCurrentCanState > STATE_SYNCED) {
    CanFrame TxFrame = {0};
    TxFrame.identifier = 0x0C010000 + +(mcuID << 8) + vcuID;
    TxFrame.extd = 1;
    TxFrame.data_length_code = 8;
    TxFrame.data[0] = uint8_t(((uint16_t)TargetPhaseCurrent));
    TxFrame.data[1] = uint8_t(((uint16_t)TargetPhaseCurrent) >> 8);
    TxFrame.data[2] = uint8_t(((uint16_t)TargetSpeed));
    TxFrame.data[3] = uint8_t(((uint16_t)TargetSpeed) >> 8);
    TxFrame.data[4] = ControlMode;
    TxFrame.data[5] = 0xAA;  // to avoid bit-stuffing
    TxFrame.data[6] = 0xAA;
    TxFrame.data[7] = LifeSignal;
    // Accepts both pointers and references
    Serial.printf("CAN writing to DA=0x%02X from SA=0x%02X.\n", mcuID, vcuID);
    if (myESP32CAN->writeFrame(TxFrame))  // timeout defaults to 1 ms
    {
      Serial.printf("CAN written to DA=0x%02X from SA=0x%02X.\n", mcuID, vcuID);
      LifeSignal++;
      ret = true;
    }
  }
  return ret;
}

uint8_t ezkontrolVCU::GetSA(CanFrame aCanFrame) {
  uint32_t lID = aCanFrame.identifier;
  return (uint8_t)lID;
}
uint8_t ezkontrolVCU::GetPS(CanFrame aCanFrame) {
  uint32_t lID = aCanFrame.identifier & 0x0000FF00;
  return (uint8_t)(lID >> 8);
}
uint8_t ezkontrolVCU::GetPF(CanFrame aCanFrame) {
  uint32_t lID = aCanFrame.identifier & 0x00FF0000;
  return (uint8_t)(lID >> 16);
}
