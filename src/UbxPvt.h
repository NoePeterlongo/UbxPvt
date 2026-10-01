#ifndef UBX_PVT_H
#define UBX_PVT_H

#include <Arduino.h>

#include "UbxPvtParser.h"

// Read-only Arduino wrapper for UbxPvtParser: reads a u-blox GNSS receiver on
// any Stream (HardwareSerial, SoftwareSerial, USB CDC, ...) and keeps the last
// NAV-PVT solution available. No task, no timer, no blocking, nothing written
// to the receiver: the caller polls as often as it wants.
//
//   Serial1.begin(115200, SERIAL_8N1, /*rx*/ 2, /*tx*/ 3); // pin routing is
//   UbxPvt gps(Serial1);                                   // your sketch's job
//   if (gps.poll()) { /* gps.pvt() is fresh */ }
class UbxPvt
{
      public:
	explicit UbxPvt(Stream &stream) : _stream(stream) {}

	// Drains the RX buffer. Returns true if at least one NAV-PVT was
	// decoded (if several arrived, pvt() holds the latest).
	bool poll()
	{
		bool received  = false;
		int  available = _stream.available();
		while (available-- > 0) {
			if (_parser.write((uint8_t)_stream.read()))
				received = true;
		}
		return received;
	}

	// Last decoded NAV-PVT.
	const UbxNavPvt    &pvt() const { return _parser.data(); }
	const UbxPvtParser &parser() const { return _parser; }

	// Registers a handler called for every valid frame (NAV-PVT included).
	// See UbxFrameHandler in UbxPvtParser.h; unregister with
	// onFrame(nullptr).
	void onFrame(UbxFrameHandler handler, void *context = nullptr)
	{
		_parser.onFrame(handler, context);
	}

      private:
	Stream      &_stream;
	UbxPvtParser _parser;
};

#endif // UBX_PVT_H
