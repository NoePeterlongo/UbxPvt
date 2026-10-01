#ifndef UBX_PVT_PARSER_H
#define UBX_PVT_PARSER_H

#include <stdint.h>
#include <stddef.h>

enum UbxPvtFixType : uint8_t {
  UBX_PVT_NO_FIX = 0,
  UBX_PVT_DEAD_RECKONING = 1,
  UBX_PVT_FIX_2D = 2,
  UBX_PVT_FIX_3D = 3,
  UBX_PVT_GNSS_PLUS_DR = 4,
  UBX_PVT_TIME_ONLY = 5,
};

// Decoded content of a UBX-NAV-PVT message (u-blox protocol 18+, payload 92 bytes).
// Raw fields keep the scaling of the protocol: lon/lat in 1e-7 deg, heights in mm, speeds in mm/s.
struct UbxNavPvt {
  uint32_t iTOW;              // ms, GPS time of week of the navigation epoch
  uint16_t year;              // UTC
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t valid;              // validity flags
  uint32_t tAcc;              // ns, time accuracy estimate
  int32_t nano;               // ns, fraction of second (-1e9..1e9)
  uint8_t fixType;            // see UbxPvtFixType
  uint8_t flags;              // fix status flags
  uint8_t numSV;              // satellites used
  int32_t lon;                // 1e-7 deg
  int32_t lat;                // 1e-7 deg
  int32_t height;             // mm, above ellipsoid
  int32_t hMSL;               // mm, above mean sea level
  uint32_t hAcc;              // mm
  uint32_t vAcc;              // mm
  int32_t velN;               // mm/s, NED
  int32_t velE;               // mm/s
  int32_t velD;               // mm/s
  int32_t gSpeed;             // mm/s, 2D ground speed
  int32_t heading;            // 1e-5 deg, heading of motion
  uint32_t sAcc;              // mm/s, speed accuracy
  uint32_t headingAcc;        // 1e-5 deg
  uint16_t pDOP;              // 0.01

  bool validDate() const { return valid & 0x01; }
  bool validTime() const { return valid & 0x02; }
  bool fullyResolved() const { return valid & 0x04; }
  bool fixOk() const { return flags & 0x01; }
  bool hasFix() const { return fixOk() && (fixType == UBX_PVT_FIX_2D || fixType == UBX_PVT_FIX_3D || fixType == UBX_PVT_GNSS_PLUS_DR); }
  bool has3DFix() const { return fixOk() && (fixType == UBX_PVT_FIX_3D || fixType == UBX_PVT_GNSS_PLUS_DR); }

  double longitudeDeg() const { return lon * 1e-7; }
  double latitudeDeg() const { return lat * 1e-7; }
  double altitudeMsl() const { return hMSL * 1e-3; }
  double groundSpeedMps() const { return gSpeed * 1e-3; }
  double headingDeg() const { return heading * 1e-5; }
};

// Called for every frame whose checksum is valid (NAV-PVT included), when registered.
// The payload buffer is owned by the parser: copy what you need, do not feed the parser from it.
typedef void (*UbxFrameHandler)(uint8_t msgClass, uint8_t msgId,
                                const uint8_t* payload, uint16_t length, void* context);

// Incremental, read-only UBX frame parser extracting NAV-PVT messages from a byte stream.
// Feed it one byte at a time or in chunks; it never blocks and never allocates.
// Frames that are not NAV-PVT are ignored (and counted), unless an onFrame() handler is
// registered: it then receives every valid frame up to MaxFrameLen bytes. A new 0xB5 0x62
// sync pair always takes precedence over a frame being ignored, so the parser re-locks
// quickly even on a noisy link.
class UbxPvtParser {
public:
  static constexpr uint8_t NavClass = 0x01;
  static constexpr uint8_t NavPvtId = 0x07;
  static constexpr uint16_t PvtLenV0 = 84;  // NAV-PVT of protocol versions < 18
  static constexpr uint16_t PvtLenV1 = 92;  // NAV-PVT of protocol versions >= 18 (M8 fw 2+, M9)
  static constexpr uint16_t MaxFrameLen = 1024; // longer frames are ignored, not handed to onFrame()

  UbxPvtParser() { reset(); }

  void reset() {
    _state = State::Sync1;
    _cka = 0;
    _ckb = 0;
    _msgClass = 0;
    _msgId = 0;
    _len = 0;
    _remaining = 0;
    _mode = Mode::Skip;
    _sawB5 = false;
    _decoded = 0;
    _checksumErrors = 0;
    _skipped = 0;
    _syncs = 0;
    _data = UbxNavPvt{};
  }

  void onFrame(UbxFrameHandler handler, void* context = nullptr) {
    _handler = handler;
    _context = context;
  }

  // Feed one byte. Returns true if a new valid NAV-PVT message was decoded.
  bool write(uint8_t byte) {
    switch (_state) {
    case State::Sync1:
      if (byte == 0xB5) _state = State::Sync2;
      break;

    case State::Sync2:
      if (byte == 0x62) {
        _state = State::Class;
        _cka = 0;
        _ckb = 0;
        _syncs++;
      } else if (byte != 0xB5) {
        _state = State::Sync1;
      }
      break;

    case State::Class:
      accumulate(byte);
      _msgClass = byte;
      _state = State::Id;
      break;

    case State::Id:
      accumulate(byte);
      _msgId = byte;
      _state = State::Len1;
      break;

    case State::Len1:
      accumulate(byte);
      _len = byte;
      _state = State::Len2;
      break;

    case State::Len2: {
      accumulate(byte);
      _len |= (uint16_t)byte << 8;
      if (_msgClass == NavClass && _msgId == NavPvtId && _len >= PvtLenV0 && _len <= PvtLenV1) {
        _mode = Mode::Pvt;
        _remaining = _len;
        _state = State::Payload;
      } else if (_handler != nullptr && _len <= MaxFrameLen) {
        _mode = Mode::Frame;
        _remaining = _len;
        _state = _len > 0 ? State::Payload : State::Cka;
      } else {
        // Ignore payload + checksum without storing. The length may be garbage
        // (noise), so resync on any 0xB5 0x62 pair seen along the way.
        _mode = Mode::Skip;
        _remaining = (uint32_t)_len + 2;
        _sawB5 = false;
        _state = State::Payload;
      }
      break;
    }

    case State::Payload:
      if (_mode == Mode::Skip) {
        if (_sawB5 && byte == 0x62) {
          _state = State::Class;
          _cka = 0;
          _ckb = 0;
          _sawB5 = false;
          _syncs++;
          break;
        }
        _sawB5 = (byte == 0xB5);
        if (--_remaining == 0) {
          _state = State::Sync1;
          _skipped++;
        }
      } else {
        accumulate(byte);
        if (_mode == Mode::Pvt) _payload[_len - _remaining] = byte;
        else _frame[_len - _remaining] = byte;
        if (--_remaining == 0) _state = State::Cka;
      }
      break;

    case State::Cka:
      if (byte == _cka) {
        _state = State::Ckb;
      } else {
        _checksumErrors++;
        _state = State::Sync1;
      }
      break;

    case State::Ckb: {
      _state = State::Sync1;
      if (byte != _ckb) {
        _checksumErrors++;
        break;
      }
      bool decoded = false;
      if (_mode == Mode::Pvt) {
        commit();
        _decoded++;
        decoded = true;
      }
      if (_mode != Mode::Skip && _handler != nullptr) {
        _handler(_msgClass, _msgId, _mode == Mode::Frame ? _frame : _payload, _len, _context);
      }
      if (decoded) return true;
      break;
    }
    }
    return false;
  }

  // Feed a chunk. Returns the number of NAV-PVT messages decoded in it.
  uint32_t write(const uint8_t* bytes, size_t length) {
    uint32_t decoded = 0;
    for (size_t i = 0; i < length; i++) {
      if (write(bytes[i])) decoded++;
    }
    return decoded;
  }

  // Last decoded NAV-PVT. Stays valid until the next decoded message overwrites it.
  const UbxNavPvt& data() const { return _data; }

  uint32_t decodedCount() const { return _decoded; }
  uint32_t checksumErrorCount() const { return _checksumErrors; }
  uint32_t skippedMessageCount() const { return _skipped; } // ignored frames fully consumed
  uint32_t syncCount() const { return _syncs; }             // 0xB5 0x62 pairs seen (useful for baud probing)

private:
  enum class State : uint8_t { Sync1, Sync2, Class, Id, Len1, Len2, Payload, Cka, Ckb };
  enum class Mode : uint8_t { Pvt, Frame, Skip };

  static uint16_t readU16(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
  }

  static uint32_t readU32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }

  void accumulate(uint8_t byte) {
    _cka = (uint8_t)(_cka + byte);
    _ckb = (uint8_t)(_ckb + _cka);
  }

  void commit() {
    for (uint16_t i = _len; i < PvtLenV1; i++) _payload[i] = 0; // short v0 frames: zero the tail

    _data.iTOW = readU32(_payload + 0);
    _data.year = readU16(_payload + 4);
    _data.month = _payload[6];
    _data.day = _payload[7];
    _data.hour = _payload[8];
    _data.minute = _payload[9];
    _data.second = _payload[10];
    _data.valid = _payload[11];
    _data.tAcc = readU32(_payload + 12);
    _data.nano = (int32_t)readU32(_payload + 16);
    _data.fixType = _payload[20];
    _data.flags = _payload[21];
    _data.numSV = _payload[23];
    _data.lon = (int32_t)readU32(_payload + 24);
    _data.lat = (int32_t)readU32(_payload + 28);
    _data.height = (int32_t)readU32(_payload + 32);
    _data.hMSL = (int32_t)readU32(_payload + 36);
    _data.hAcc = readU32(_payload + 40);
    _data.vAcc = readU32(_payload + 44);
    _data.velN = (int32_t)readU32(_payload + 48);
    _data.velE = (int32_t)readU32(_payload + 52);
    _data.velD = (int32_t)readU32(_payload + 56);
    _data.gSpeed = (int32_t)readU32(_payload + 60);
    _data.heading = (int32_t)readU32(_payload + 64);
    _data.sAcc = readU32(_payload + 68);
    _data.headingAcc = readU32(_payload + 72);
    _data.pDOP = readU16(_payload + 76);
  }

  State _state;
  Mode _mode;
  uint8_t _cka;
  uint8_t _ckb;
  uint8_t _msgClass;
  uint8_t _msgId;
  uint16_t _len;
  uint32_t _remaining;
  bool _sawB5;
  uint8_t _payload[PvtLenV1];
  uint8_t _frame[MaxFrameLen];
  UbxNavPvt _data;
  UbxFrameHandler _handler = nullptr;
  void* _context = nullptr;
  uint32_t _decoded;
  uint32_t _checksumErrors;
  uint32_t _skipped;
  uint32_t _syncs;
};

#endif // UBX_PVT_PARSER_H
