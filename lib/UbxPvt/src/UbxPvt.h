#ifndef UBX_PVT_H
#define UBX_PVT_H

#include <Arduino.h>
#include <HardwareSerial.h>

#include "UbxPvtParser.h"

// Read-only Arduino wrapper for UbxPvtParser: reads a u-blox GNSS receiver on a
// hardware UART and keeps the last NAV-PVT solution available. No task, no timer,
// no blocking, nothing written to the receiver: the caller polls as often as it wants.
//
//   HardwareSerial gpsSerial(1);
//   UbxPvt gps(gpsSerial);
//   gps.begin(115200, 2, 3);   // baud, rxPin (GPS TX), txPin (GPS RX)
//   if (gps.poll()) { /* gps.pvt() is fresh */ }
class UbxPvt {
public:
  explicit UbxPvt(HardwareSerial& serial) : _serial(serial) {}

  void begin(uint32_t baud) { _serial.begin(baud); }
  void begin(uint32_t baud, int8_t rxPin, int8_t txPin) {
    _serial.begin(baud, SERIAL_8N1, rxPin, txPin);
  }

  // Drains the RX buffer. Returns true if at least one NAV-PVT was decoded
  // (if several arrived, pvt() holds the latest).
  bool poll() {
    bool received = false;
    int available = _serial.available();
    while (available-- > 0) {
      if (_parser.write((uint8_t)_serial.read())) received = true;
    }
    return received;
  }

  const UbxNavPvt& pvt() const { return _parser.data(); }
  const UbxPvtParser& parser() const { return _parser; }

  // Registers a handler called for every valid frame (NAV-PVT included).
  // See UbxFrameHandler in UbxPvtParser.h; unregister with onFrame(nullptr).
  void onFrame(UbxFrameHandler handler, void* context = nullptr) {
    _parser.onFrame(handler, context);
  }

private:
  HardwareSerial& _serial;
  UbxPvtParser _parser;
};

#endif // UBX_PVT_H
