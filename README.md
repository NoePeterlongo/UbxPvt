# UbxPvt

Read-only Arduino/PlatformIO library that reads **UBX-NAV-PVT** messages from a
u-blox GNSS receiver (M8/M9 generation, e.g. NEO-M9N) on a serial port, with an
incremental parser core and an optional generic UBX frame handler.

Validated on hardware (ESP32-S3 + NEO-M9N at 115200 baud, 10 Hz NAV-PVT stream,
zero checksum errors) and covered by 19 native unit tests.

## Features

- **Task-free and non-blocking** — `poll()` only drains the serial buffer and
  returns in microseconds. The library never creates a task, thread or timer;
  your program calls it whenever it wants.
- **Read-only** — the library never writes to the receiver. Configuration is
  done with u-center or your own code.
- **Robust** — incremental state machine, resynchronization on the `0xB5 0x62`
  sync bytes, Fletcher-8 checksum validation of every decoded or dispatched
  frame, immediate re-lock on a sync pair found inside an ignored frame, no
  dynamic allocation.
- **Extensible** — an optional `onFrame()` handler receives *every* valid frame
  (NAV-SAT, MON-HW, ACK, ...), NAV-PVT included, as a raw payload.
- **Portable** — the parser core (`UbxPvtParser.h`) is pure standard C++ with
  no Arduino dependency; the wrapper (`UbxPvt.h`) works with any Arduino
  `Stream`. Nothing is ESP32-specific.

## Compatibility

| Component | Depends on | Runs on |
| --- | --- | --- |
| `UbxPvtParser.h` | C++11 (`<stdint.h>` only) | anywhere: host unit tests, any MCU |
| `UbxPvt.h` | Arduino `Stream` | any Arduino core (ESP32, AVR, SAMD, RP2040, ...) |

Pin routing is your sketch's responsibility and uses whatever your core provides:

```cpp
Serial1.begin(115200, SERIAL_8N1, /*rx*/ 2, /*tx*/ 3); // ESP32 example
```

## Installation

**PlatformIO** (project `platformio.ini`):

```ini
lib_deps =
    UbxPvt = file:///path/to/this/repo   ; or a git URL
```

**Arduino IDE**: download or clone this repository into your `libraries/`
folder (or via *Sketch > Include Library > Add .ZIP Library*).

The repository is itself a valid Arduino/PlatformIO library layout:

```
src/UbxPvtParser.h        parser core (pure C++, header-only)
src/UbxPvt.h              Arduino Stream wrapper (header-only)
test/test_main.cpp        native unit tests (Unity, via PlatformIO)
examples/NavPvtMonitor/   demo: NAV-PVT monitor (ESP32-S3 + M9N)
examples/NavSatMonitor/   demo: NAV-SAT parser built on the onFrame() handler
```

## Wiring (example: NEO-M9N on ESP32-S3)

| GNSS module | ESP32 |
| --- | --- |
| TX | GPIO2 (UART1 RX) |
| RX | GPIO3 (UART1 TX — optional, unused by the library) |
| GND | GND |

## Quick start

```cpp
#include <UbxPvt.h>

UbxPvt gps(Serial1);

void setup() {
  Serial.begin(115200);
  // Pin routing is the sketch's job, using whatever the core provides:
  Serial1.begin(115200, SERIAL_8N1, /*rx=*/2, /*tx=*/3); // ESP32 example
}

void loop() {
  if (gps.poll()) { // true: a new NAV-PVT was just decoded
    const UbxNavPvt &pvt = gps.pvt();
    if (pvt.has3DFix()) {
      Serial.printf("lat %.7f lon %.7f alt %.1f m speed %.2f m/s %02u:%02u:%02u UTC\n",
                    pvt.latitudeDeg(), pvt.longitudeDeg(), pvt.altitudeMsl(),
                    pvt.groundSpeedMps(), pvt.hour, pvt.minute, pvt.second);
    }
  }
}
```

To decode other message types as well, add a frame handler and register it in
`setup()` — see `examples/NavSatMonitor/` for a complete NAV-SAT decoder:

```cpp
void onFrame(uint8_t msgClass, uint8_t msgId, const uint8_t *payload,
             uint16_t len, void *ctx) {
  if (msgClass == 0x01 && msgId == 0x35) { /* NAV-SAT: constellation */ }
}

// in setup(), after Serial1.begin():
gps.onFrame(onFrame); // gps.onFrame(nullptr) to unregister
```

## Examples

Two runnable hardware demos, each a standalone PlatformIO project:

- `examples/NavPvtMonitor/` — NAV-PVT monitor: baud-rate scan, once-per-second
  solution printing, link statistics and a per-frame-type breakdown with RF
  diagnostics (tracked satellites, mean C/N0, GNSS core noise level).
- `examples/NavSatMonitor/` — how to build a parser for **another message
  type**: a self-contained UBX-NAV-SAT decoder (per-satellite GNSS system,
  C/N0, elevation, azimuth, used-in-solution flag) fed from the raw payload
  delivered by the `onFrame()` handler, next to the built-in NAV-PVT.

```sh
cd examples/NavPvtMonitor   # or examples/NavSatMonitor
pio run -t upload            # adjust the environment to your board
pio device monitor           # 115200 baud
```

## API

### `UbxPvt` — Arduino wrapper (`src/UbxPvt.h`)

| Method | Description |
| --- | --- |
| `UbxPvt(Stream &stream)` | wraps any serial-ish `Stream` |
| `poll()` | drains the RX buffer; `true` if at least one NAV-PVT was decoded |
| `pvt()` | `const UbxNavPvt &` to the last decoded message |
| `parser()` | `const UbxPvtParser &` for statistics |
| `onFrame(handler, ctx = nullptr)` | handler called for every valid frame |

### `UbxNavPvt` — decoded solution (`src/UbxPvtParser.h`)

Attributes (raw values, protocol scaling preserved):

| Attribute | Type | Unit / scaling | Description |
| --- | --- | --- | --- |
| `iTOW` | `uint32_t` | ms | GPS time of week of the navigation epoch |
| `year` | `uint16_t` | — | UTC year |
| `month` | `uint8_t` | — | UTC month, 1..12 |
| `day` | `uint8_t` | — | UTC day of month, 1..31 |
| `hour` | `uint8_t` | — | UTC hour, 0..23 |
| `minute` | `uint8_t` | — | UTC minute, 0..59 |
| `second` | `uint8_t` | — | UTC second, 0..60 |
| `valid` | `uint8_t` | flags | validity flags (date / time / fully resolved) |
| `tAcc` | `uint32_t` | ns | time accuracy estimate |
| `nano` | `int32_t` | ns | sub-second part of UTC time (-1e9..1e9) |
| `fixType` | `uint8_t` | — | fix type, see `UbxPvtFixType` below |
| `flags` | `uint8_t` | flags | fix status flags (bit 0: gnssFixOk) |
| `numSV` | `uint8_t` | — | satellites used in the solution |
| `lon` | `int32_t` | 1e-7 deg | longitude |
| `lat` | `int32_t` | 1e-7 deg | latitude |
| `height` | `int32_t` | mm | height above ellipsoid |
| `hMSL` | `int32_t` | mm | height above mean sea level |
| `hAcc` | `uint32_t` | mm | horizontal accuracy estimate |
| `vAcc` | `uint32_t` | mm | vertical accuracy estimate |
| `velN` | `int32_t` | mm/s | velocity, north component (NED) |
| `velE` | `int32_t` | mm/s | velocity, east component (NED) |
| `velD` | `int32_t` | mm/s | velocity, down component (NED) |
| `gSpeed` | `int32_t` | mm/s | 2D ground speed |
| `heading` | `int32_t` | 1e-5 deg | heading of motion |
| `sAcc` | `uint32_t` | mm/s | speed accuracy estimate |
| `headingAcc` | `uint32_t` | 1e-5 deg | heading accuracy estimate |
| `pDOP` | `uint16_t` | 0.01 | position DOP |

Methods:

| Method | Returns | Description |
| --- | --- | --- |
| `validDate()` | `bool` | date is valid (`valid` bit 0) |
| `validTime()` | `bool` | time is valid (`valid` bit 1) |
| `fullyResolved()` | `bool` | time fully resolved (`valid` bit 2) |
| `fixOk()` | `bool` | fix is valid (`flags` bit 0, gnssFixOk) |
| `hasFix()` | `bool` | valid 2D, 3D or GNSS+DR fix |
| `has3DFix()` | `bool` | valid 3D or GNSS+DR fix |
| `longitudeDeg()` | `double` | longitude in degrees |
| `latitudeDeg()` | `double` | latitude in degrees |
| `altitudeMsl()` | `double` | height above sea level in metres |
| `groundSpeedMps()` | `double` | ground speed in m/s |
| `headingDeg()` | `double` | heading of motion in degrees |

`fixType` values (`UbxPvtFixType`):

| Constant | Value | Meaning |
| --- | --- | --- |
| `UBX_PVT_NO_FIX` | 0 | no fix |
| `UBX_PVT_DEAD_RECKONING` | 1 | dead reckoning only |
| `UBX_PVT_FIX_2D` | 2 | 2D fix |
| `UBX_PVT_FIX_3D` | 3 | 3D fix |
| `UBX_PVT_GNSS_PLUS_DR` | 4 | GNSS + dead reckoning combined |
| `UBX_PVT_TIME_ONLY` | 5 | time only fix |

### `UbxPvtParser` — portable core

| Member | Description |
| --- | --- |
| `write(byte)` / `write(bytes, len)` | feed bytes; returns true / count when NAV-PVT decoded |
| `data()` | last decoded `UbxNavPvt` |
| `onFrame(handler, ctx = nullptr)` | handler for every valid frame up to 1024 bytes |
| `reset()` | clears state, counters and `data()` |
| `decodedCount()` | valid NAV-PVT messages decoded |
| `checksumErrorCount()` | frames rejected by checksum |
| `skippedMessageCount()` | ignored frames fully consumed |
| `syncCount()` | `0xB5 0x62` pairs seen (useful for baud probing) |

Handler contract: called synchronously from `write()`/`poll()`, for every frame
whose checksum is valid (NAV-PVT included). The payload buffer belongs to the
parser — copy what you need, and do not feed the parser from the callback.
Frames longer than 1024 bytes are consumed and ignored, not dispatched.

## Testing

Unit tests (no hardware, run on the development machine):

```sh
pio test -e native
```

19 test cases: field-by-field decoding, checksum validation, resynchronization
after garbage / NMEA / truncated frames, interleaved non-PVT frames,
byte-by-byte feeding, 84- and 92-byte payloads, pseudo-random noise fuzz, and
the generic handler (dispatch, context, bad checksum, oversized frame, empty
frame, unregistration). The same suite runs on every push through
`.github/workflows/ci.yml`.

For hardware validation, build and flash one of the bundled examples — see
[Examples](#examples). Both were exercised against an ESP32-S3 and a NEO-M9N
(115200 baud, 10 Hz NAV-PVT stream, zero checksum errors).

## Protocol notes

- UBX-NAV-PVT payload: 92 bytes for protocol version >= 18 (M8 firmware 2+,
  M9), 84 bytes before; both are accepted.
- On u-blox 9 (protocol 27+) the header layout of some messages differs from
  M8 documentation: NAV-SAT starts with `iTOW` (U4), then `version` (U1) and
  `numSvs` (U1, offset 5), with 12-byte items from offset 8; MON-HW no longer
  reports antenna status (moved to MON-RF) and carries `noisePerMS` at offset
  16. The demo's handler shows the correct offsets.

## Limitations

- Only NAV-PVT is decoded into fields; other frames are delivered raw through
  `onFrame()`.
- Without `onFrame()`, ignored frames are consumed without checksum
  verification (only decoded NAV-PVT frames are validated).
- No ACK handling and no receiver configuration (read-only library).

## License

MIT — see [LICENSE](LICENSE).
