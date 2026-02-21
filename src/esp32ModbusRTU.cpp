/* esp32ModbusRTU

Copyright 2018 Bert Melis

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "esp32ModbusRTU.h"
#include "sensesp.h"
#if defined ARDUINO_ARCH_ESP32

using namespace esp32ModbusRTUInternals;  // NOLINT

esp32ModbusRTU::esp32ModbusRTU(HardwareSerial* serial, int8_t txdPin, int8_t rxdPin, int8_t rtsPin) :
  TimeOutValue(TIMEOUT_MS),
  _serial(serial),
  _lastMillis(0),
  _interval(0),
  _rtsPin(rtsPin),
  _txdPin(txdPin),
  _rxdPin(rxdPin),
  _task(nullptr),
  _queue(nullptr)  {
    _queue = xQueueCreate(QUEUE_SIZE, sizeof(ModbusRequest*));
}

esp32ModbusRTU::~esp32ModbusRTU() {
  // TODO(bertmelis): kill task and cleanup
}

void esp32ModbusRTU::begin(int coreID /* = -1 */) {
  debugD("[MODBUS] begin() called");
  
  // If rtsPin is >=0, the RS485 adapter needs send/receive toggle
  if (_rtsPin >= 0) {
    debugD("[MODBUS] Setting RTS pin %d to OUTPUT", _rtsPin);
    pinMode(_rtsPin, OUTPUT);
    digitalWrite(_rtsPin, LOW);
  }
  
  // Ensure queue is created (may have failed at global init time if RTOS wasn't ready)
  if (_queue == nullptr) {
    debugD("[MODBUS] Creating queue...");
    _queue = xQueueCreate(QUEUE_SIZE, sizeof(ModbusRequest*));
    if (_queue == nullptr) {
      // Queue creation failed - critical error
      debugD("[MODBUS] ERROR: Queue creation failed!");
      return;
    }
  }
  
  debugD("[MODBUS] Setting serial pins: RX=%d, TX=%d", _rxdPin, _txdPin);
  _serial->setPins(_rxdPin, _txdPin, -1, -1);
  
  debugD("[MODBUS] Creating handler task...");
  xTaskCreatePinnedToCore((TaskFunction_t)&_handleConnection, "esp32ModbusRTU", 4096, this, 5, &_task, coreID >= 0 ? coreID : NULL);
  
  debugD("[MODBUS] Task created, setting interval");
  // silent interval is at least 3.5x character time
  _interval = 5;
  if (_interval == 0) _interval = 1;  // minimum of 1msec interval
  debugD("[MODBUS] begin() complete, interval=%d ms", _interval);
}

bool esp32ModbusRTU::readDiscreteInputs(uint8_t slaveAddress, uint16_t address, uint16_t numberCoils) {
  ModbusRequest* request = new ModbusRequest02(slaveAddress, address, numberCoils);
  return _addToQueue(request);
}
bool esp32ModbusRTU::readHoldingRegisters(uint8_t slaveAddress, uint16_t address, uint16_t numberRegisters) {
  ModbusRequest* request = new ModbusRequest03(slaveAddress, address, numberRegisters);
  return _addToQueue(request);
}

bool esp32ModbusRTU::readInputRegisters(uint8_t slaveAddress, uint16_t address, uint16_t numberRegisters) {
  ModbusRequest* request = new ModbusRequest04(slaveAddress, address, numberRegisters);
  return _addToQueue(request);
}

bool esp32ModbusRTU::writeSingleHoldingRegister(uint8_t slaveAddress, uint16_t address, uint16_t data) {
  ModbusRequest* request = new ModbusRequest06(slaveAddress, address, data);
  return _addToQueue(request);
}

bool esp32ModbusRTU::writeMultHoldingRegisters(uint8_t slaveAddress, uint16_t address, uint16_t numberRegisters, uint8_t* data) {
  ModbusRequest* request = new ModbusRequest16(slaveAddress, address, numberRegisters, data);
  return _addToQueue(request);
}

void esp32ModbusRTU::onData(esp32Modbus::MBRTUOnData handler) {
 _onData = handler;
}

void esp32ModbusRTU::onError(esp32Modbus::MBRTUOnError handler) {
 _onError = handler;
}

bool esp32ModbusRTU::_addToQueue(ModbusRequest* request) {
  if (!request || !_queue) {
    if (request) delete request;
    return false;
  } else if (xQueueSend(_queue, reinterpret_cast<void*>(&request), (TickType_t)0) != pdPASS) {
    delete request;
    return false;
  } else {
    return true;
  }
}

void esp32ModbusRTU::_handleConnection(esp32ModbusRTU* instance) {
  debugD("[MODBUS] Handler task started");
  while (1) {
    ModbusRequest* request;
    if (pdTRUE == xQueueReceive(instance->_queue, &request, portMAX_DELAY)) {  // block and wait for queued item
      instance->_send(request->getMessage(), request->getSize());
       ModbusResponse* response = instance->_receive(request);
      if (response->isSucces()) {
        if (instance->_onData) instance->_onData(response->getSlaveAddress(), response->getFunctionCode(), request->getAddress(), response->getData(), response->getByteCount());
       } else {
        if (instance->_onError) instance->_onError(response->getError());
     }
      delete request;  // object created in public methods
      delete response;  // object created in _receive()
    }
  }
}

void esp32ModbusRTU::_send(uint8_t* data, uint8_t length) {
  // Clear RX buffer before sending to avoid contamination from previous messages
  //debugD("[MODBUS] Clearing RX buffer");
  while (_serial->available()) {
    _serial->read();
  }
  
  while (millis() - _lastMillis < _interval) delay(1);  // respect _interval
  // Toggle rtsPin, if necessary
  if (_rtsPin >= 0) {
    digitalWrite(_rtsPin, HIGH);
  }
  size_t written = _serial->write(data, length);
  _serial->flush();
  // Toggle rtsPin, if necessary
  if (_rtsPin >= 0) {
    digitalWrite(_rtsPin, LOW);
  }
  _lastMillis = millis();
}

// Adjust timeout on MODBUS - some slaves require longer/allow for shorter times
void esp32ModbusRTU::setTimeOutValue(uint32_t tov) {
  if (tov) TimeOutValue = tov;
}

ModbusResponse* esp32ModbusRTU::_receive(ModbusRequest* request) {
  ModbusResponse* response = new ModbusResponse(request->responseLength(), request);
  //debugD("[MODBUS] _receive() waiting for response...");
  uint32_t startTime = millis();
  uint32_t lastByteTime = millis();
  bool firstByteReceived = false;
  
  while (true) {
    if (_serial->available()) {
      uint8_t byte = _serial->read();
      if (!firstByteReceived) {
        debugI("[MODBUS] First byte received after %d ms (0x%02x)", millis() - startTime, byte);
      }
      //debugD("[MODBUS] Received byte: 0x%02x", byte);
      response->add(byte);
      lastByteTime = millis();
      firstByteReceived = true;
    }
    
    // Keep reading for 200ms after the first byte starts arriving, then give 100ms grace period for stragglers
    if (firstByteReceived && (millis() - lastByteTime > 100)) {
      debugI("[MODBUS] Response complete! Got %d bytes (waited %d ms for full response)", response->getSize(), millis() - startTime);
      _lastMillis = millis();
      break;
    }
    
    if (millis() - _lastMillis > TimeOutValue) {
      if (!firstByteReceived) {
        debugW("[MODBUS] NO DATA RECEIVED - Timeout after %d ms (TimeOutValue=%d ms)", millis() - startTime, TimeOutValue);
      } else {
        debugD("[MODBUS] Timeout waiting for response (waited %d ms)", millis() - startTime);
      }
      break;
    }
    delay(1);  // take care of watchdog
  }
  return response;
}

#elif defined ESP32MODBUSRTU_TEST

#else

#pragma message "no suitable platform"

#endif
