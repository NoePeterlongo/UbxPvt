#include <unity.h>

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <vector>

#include "../lib/UbxPvt/src/UbxPvtParser.h"

static std::vector<uint8_t> frame(uint8_t msgClass, uint8_t msgId, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> f = {0xB5, 0x62, msgClass, msgId,
                            (uint8_t)payload.size(), (uint8_t)(payload.size() >> 8)};
  uint8_t cka = 0, ckb = 0;
  for (size_t i = 2; i < 6; i++) {
    cka += f[i];
    ckb += cka;
  }
  for (uint8_t b : payload) {
    cka += b;
    ckb += cka;
    f.push_back(b);
  }
  f.push_back(cka);
  f.push_back(ckb);
  return f;
}

static void put32(std::vector<uint8_t>& v, size_t off, uint32_t val) {
  v[off] = (uint8_t)val;
  v[off + 1] = (uint8_t)(val >> 8);
  v[off + 2] = (uint8_t)(val >> 16);
  v[off + 3] = (uint8_t)(val >> 24);
}

static void put16(std::vector<uint8_t>& v, size_t off, uint16_t val) {
  v[off] = (uint8_t)val;
  v[off + 1] = (uint8_t)(val >> 8);
}

static std::vector<uint8_t> samplePvt(uint32_t iTOW, uint8_t fixType = UBX_PVT_FIX_3D) {
  std::vector<uint8_t> p(92, 0);
  put32(p, 0, iTOW);                                  // iTOW
  put16(p, 4, 2026);                                  // year
  p[6] = 10;                                          // month
  p[7] = 1;                                           // day
  p[8] = 12; p[9] = 34; p[10] = 56;                   // hour min sec
  p[11] = 0x07;                                       // valid: date + time + fully resolved
  put32(p, 12, 1000);                                 // tAcc
  put32(p, 16, (uint32_t)-123456789);                 // nano
  p[20] = fixType;                                    // fixType
  p[21] = 0x01;                                       // flags: gnssFixOK
  p[23] = 12;                                         // numSV
  put32(p, 24, 488583700);                            // lon 48.8583700 deg
  put32(p, 28, 22944800);                             // lat 2.2944800 deg
  put32(p, 32, 42100);                                // height 42.1 m
  put32(p, 36, 35000);                                // hMSL 35.0 m
  put32(p, 40, 1500);                                 // hAcc
  put32(p, 44, 2000);                                 // vAcc
  put32(p, 48, 100);                                  // velN
  put32(p, 52, (uint32_t)-50);                        // velE
  put32(p, 56, 25);                                   // velD
  put32(p, 60, 112);                                  // gSpeed
  put32(p, 64, 4567890);                             // heading 45.67890 deg
  put32(p, 68, 300);                                  // sAcc
  put32(p, 72, 200000);                               // headingAcc
  put16(p, 76, 120);                                  // pDOP 1.20
  return p;
}

static void checkSample(const UbxNavPvt& pvt) {
  TEST_ASSERT_EQUAL_UINT32(0x11223344, pvt.iTOW);
  TEST_ASSERT_EQUAL_UINT16(2026, pvt.year);
  TEST_ASSERT_EQUAL_UINT8(10, pvt.month);
  TEST_ASSERT_EQUAL_UINT8(1, pvt.day);
  TEST_ASSERT_EQUAL_UINT8(12, pvt.hour);
  TEST_ASSERT_EQUAL_UINT8(34, pvt.minute);
  TEST_ASSERT_EQUAL_UINT8(56, pvt.second);
  TEST_ASSERT_EQUAL_UINT8(0x07, pvt.valid);
  TEST_ASSERT_EQUAL_UINT32(1000, pvt.tAcc);
  TEST_ASSERT_EQUAL_INT32(-123456789, pvt.nano);
  TEST_ASSERT_EQUAL_UINT8(UBX_PVT_FIX_3D, pvt.fixType);
  TEST_ASSERT_EQUAL_UINT8(0x01, pvt.flags);
  TEST_ASSERT_EQUAL_UINT8(12, pvt.numSV);
  TEST_ASSERT_EQUAL_INT32(488583700, pvt.lon);
  TEST_ASSERT_EQUAL_INT32(22944800, pvt.lat);
  TEST_ASSERT_EQUAL_INT32(42100, pvt.height);
  TEST_ASSERT_EQUAL_INT32(35000, pvt.hMSL);
  TEST_ASSERT_EQUAL_UINT32(1500, pvt.hAcc);
  TEST_ASSERT_EQUAL_UINT32(2000, pvt.vAcc);
  TEST_ASSERT_EQUAL_INT32(100, pvt.velN);
  TEST_ASSERT_EQUAL_INT32(-50, pvt.velE);
  TEST_ASSERT_EQUAL_INT32(25, pvt.velD);
  TEST_ASSERT_EQUAL_INT32(112, pvt.gSpeed);
  TEST_ASSERT_EQUAL_INT32(4567890, pvt.heading);
  TEST_ASSERT_EQUAL_UINT32(300, pvt.sAcc);
  TEST_ASSERT_EQUAL_UINT32(200000, pvt.headingAcc);
  TEST_ASSERT_EQUAL_UINT16(120, pvt.pDOP);
}

static void checkHelpers(const UbxNavPvt& pvt) {
  TEST_ASSERT_TRUE(pvt.validDate());
  TEST_ASSERT_TRUE(pvt.validTime());
  TEST_ASSERT_TRUE(pvt.fullyResolved());
  TEST_ASSERT_TRUE(pvt.fixOk());
  TEST_ASSERT_TRUE(pvt.hasFix());
  TEST_ASSERT_TRUE(pvt.has3DFix());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 48.8583700, pvt.longitudeDeg());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.2944800, pvt.latitudeDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.001, 35.0, pvt.altitudeMsl());
  TEST_ASSERT_FLOAT_WITHIN(0.001, 0.112, pvt.groundSpeedMps());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 45.67890, pvt.headingDeg());
}

static void test_decode_valid_frame(void) {
  UbxPvtParser parser;
  auto f = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(0x11223344));
  TEST_ASSERT_EQUAL_UINT32(1, parser.write(f.data(), f.size()));
  TEST_ASSERT_EQUAL_UINT32(1, parser.decodedCount());
  TEST_ASSERT_EQUAL_UINT32(0, parser.checksumErrorCount());
  checkSample(parser.data());
  checkHelpers(parser.data());
}

static void test_decode_byte_by_byte(void) {
  UbxPvtParser parser;
  auto f = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(0x11223344));
  bool decoded = false;
  for (uint8_t b : f) decoded |= parser.write(b);
  TEST_ASSERT_TRUE(decoded);
  checkSample(parser.data());
}

static void test_checksum_error_then_resync(void) {
  UbxPvtParser parser;
  auto f = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(1));
  f[20] ^= 0xFF; // corrupt one payload byte
  TEST_ASSERT_EQUAL_UINT32(0, parser.write(f.data(), f.size()));
  TEST_ASSERT_EQUAL_UINT32(1, parser.checksumErrorCount());
  auto g = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(2));
  TEST_ASSERT_EQUAL_UINT32(1, parser.write(g.data(), g.size()));
  TEST_ASSERT_EQUAL_UINT32(1, parser.decodedCount());
  TEST_ASSERT_EQUAL_UINT32(2, parser.data().iTOW);
}

static void test_other_ubx_messages_are_skipped(void) {
  UbxPvtParser parser;
  std::vector<uint8_t> posllh(28, 0x55); // NAV-POSLLH, class 0x01 id 0x02
  auto other = frame(0x01, 0x02, posllh);
  auto pvt = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(7));
  std::vector<uint8_t> stream;
  stream.insert(stream.end(), other.begin(), other.end());
  stream.insert(stream.end(), pvt.begin(), pvt.end());
  TEST_ASSERT_EQUAL_UINT32(1, parser.write(stream.data(), stream.size()));
  TEST_ASSERT_EQUAL_UINT32(1, parser.skippedMessageCount());
  TEST_ASSERT_EQUAL_UINT32(7, parser.data().iTOW);
}

static void test_noise_nMEA_and_partial_frames(void) {
  UbxPvtParser parser;
  auto good = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(9));
  std::vector<uint8_t> stream = {
      0xFF, 0x00, 0xB5,                                             // garbage + lone sync byte
      '$', 'G', 'P', 'G', 'G', 'A', ',', 0xB5, 0x62, 0x01, 0x07,    // NMEA with a fake header inside
      0xB5, 0x62, 0x01, 0x07, 0x5C, 0x00,                           // truncated frame
  };
  stream.insert(stream.end(), good.begin(), good.end());
  TEST_ASSERT_EQUAL_UINT32(1, parser.write(stream.data(), stream.size()));
  TEST_ASSERT_EQUAL_UINT32(1, parser.decodedCount());
  TEST_ASSERT_EQUAL_UINT32(9, parser.data().iTOW);
}

static void test_two_frames_back_to_back(void) {
  UbxPvtParser parser;
  auto a = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(100));
  auto b = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(101));
  std::vector<uint8_t> stream;
  stream.insert(stream.end(), a.begin(), a.end());
  stream.insert(stream.end(), b.begin(), b.end());
  TEST_ASSERT_EQUAL_UINT32(2, parser.write(stream.data(), stream.size()));
  TEST_ASSERT_EQUAL_UINT32(2, parser.decodedCount());
  TEST_ASSERT_EQUAL_UINT32(101, parser.data().iTOW); // latest wins
}

static void test_short_v0_payload_accepted(void) {
  UbxPvtParser parser;
  std::vector<uint8_t> payload = samplePvt(5);
  payload.resize(UbxPvtParser::PvtLenV0); // protocol < 18: 84 bytes
  auto f = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, payload);
  TEST_ASSERT_EQUAL_UINT32(1, parser.write(f.data(), f.size()));
  TEST_ASSERT_EQUAL_UINT32(488583700, parser.data().lon);
}

static void test_oversized_pvt_is_skipped(void) {
  UbxPvtParser parser;
  std::vector<uint8_t> payload(100, 0x11);
  auto f = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, payload);
  TEST_ASSERT_EQUAL_UINT32(0, parser.write(f.data(), f.size()));
  TEST_ASSERT_EQUAL_UINT32(0, parser.decodedCount());
  TEST_ASSERT_EQUAL_UINT32(1, parser.skippedMessageCount());
}

static void test_no_fix_flags(void) {
  UbxPvtParser parser;
  auto payload = samplePvt(1, UBX_PVT_NO_FIX);
  payload[21] = 0x00; // gnssFixOK cleared
  auto f = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, payload);
  TEST_ASSERT_EQUAL_UINT32(1, parser.write(f.data(), f.size()));
  TEST_ASSERT_EQUAL_UINT8(UBX_PVT_NO_FIX, parser.data().fixType);
  TEST_ASSERT_FALSE(parser.data().fixOk());
  TEST_ASSERT_FALSE(parser.data().hasFix());
  TEST_ASSERT_FALSE(parser.data().has3DFix());
}

static void test_reset(void) {
  UbxPvtParser parser;
  auto f = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(1));
  parser.write(f.data(), f.size());
  parser.reset();
  TEST_ASSERT_EQUAL_UINT32(0, parser.decodedCount());
  TEST_ASSERT_EQUAL_UINT32(0, parser.checksumErrorCount());
  TEST_ASSERT_EQUAL_UINT32(0, parser.syncCount());
  TEST_ASSERT_EQUAL_UINT32(0, parser.data().iTOW);
  // parser is usable again right after reset
  TEST_ASSERT_EQUAL_UINT32(1, parser.write(f.data(), f.size()));
}

static void test_sync_count(void) {
  UbxPvtParser parser;
  auto f = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(1));
  parser.write(f.data(), f.size());
  TEST_ASSERT_EQUAL_UINT32(1, parser.syncCount());
}

static void test_fuzz_noise_recovery(void) {
  UbxPvtParser parser;
  uint32_t seed = 12345; // deterministic LCG noise
  std::vector<uint8_t> stream;
  for (int i = 0; i < 4096; i++) {
    seed = seed * 1664525u + 1013904223u;
    stream.push_back((uint8_t)(seed >> 16));
  }
  auto good = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(42));
  stream.insert(stream.end(), good.begin(), good.end());
  TEST_ASSERT_EQUAL_UINT32(1, parser.write(stream.data(), stream.size()));
  TEST_ASSERT_EQUAL_UINT32(42, parser.data().iTOW);
}

static void test_sync_pair_inside_skipped_payload(void) {
  UbxPvtParser parser;
  std::vector<uint8_t> payload(28, 0x00); // NAV-POSLLH with 0xB5 0x62 inside
  payload[10] = 0xB5;
  payload[11] = 0x62;
  auto other = frame(0x01, 0x02, payload);
  auto good = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(3));
  std::vector<uint8_t> stream;
  stream.insert(stream.end(), other.begin(), other.end());
  stream.insert(stream.end(), good.begin(), good.end());
  TEST_ASSERT_EQUAL_UINT32(1, parser.write(stream.data(), stream.size()));
  TEST_ASSERT_EQUAL_UINT32(1, parser.decodedCount());
  TEST_ASSERT_EQUAL_UINT32(3, parser.data().iTOW);
}

// --- generic onFrame handler ---

struct Recorded {
  uint8_t cls;
  uint8_t id;
  uint16_t len;
  uint8_t payload[128];
  uint32_t calls;
  void* context;
};

static Recorded rec;

static void record(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t length, void* context) {
  rec.cls = msgClass;
  rec.id = msgId;
  rec.len = length;
  uint16_t n = length < (uint16_t)sizeof(rec.payload) ? length : (uint16_t)sizeof(rec.payload);
  if (n > 0) memcpy(rec.payload, payload, n);
  rec.calls++;
  rec.context = context;
}

static void test_handler_receives_pvt_and_other_frames(void) {
  UbxPvtParser parser;
  memset(&rec, 0, sizeof(rec));
  parser.onFrame(record, &rec);

  auto posllh = frame(0x01, 0x02, std::vector<uint8_t>(28, 0x55));
  TEST_ASSERT_EQUAL_UINT32(0, parser.write(posllh.data(), posllh.size()));
  TEST_ASSERT_EQUAL_UINT32(1, rec.calls);
  TEST_ASSERT_EQUAL_UINT8(0x01, rec.cls);
  TEST_ASSERT_EQUAL_UINT8(0x02, rec.id);
  TEST_ASSERT_EQUAL_UINT16(28, rec.len);
  TEST_ASSERT_EQUAL_UINT8(0x55, rec.payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0x55, rec.payload[27]);
  TEST_ASSERT_EQUAL_PTR(&rec, rec.context);

  auto pvt = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(77));
  TEST_ASSERT_EQUAL_UINT32(1, parser.write(pvt.data(), pvt.size()));
  TEST_ASSERT_EQUAL_UINT32(2, rec.calls);
  TEST_ASSERT_EQUAL_UINT8(0x07, rec.id);
  TEST_ASSERT_EQUAL_UINT16(92, rec.len);
  TEST_ASSERT_EQUAL_UINT32(77, (uint32_t)rec.payload[0] | ((uint32_t)rec.payload[1] << 8) |
                                   ((uint32_t)rec.payload[2] << 16) | ((uint32_t)rec.payload[3] << 24));
  TEST_ASSERT_EQUAL_UINT32(77, parser.data().iTOW);
}

static void test_handler_context_pointer(void) {
  UbxPvtParser parser;
  memset(&rec, 0, sizeof(rec));
  parser.onFrame(record, &rec); // context is &rec
  auto pvt = frame(UbxPvtParser::NavClass, UbxPvtParser::NavPvtId, samplePvt(1));
  parser.write(pvt.data(), pvt.size());
  TEST_ASSERT_EQUAL_PTR(&rec, rec.context);
}

static void test_handler_ignores_bad_checksum(void) {
  UbxPvtParser parser;
  memset(&rec, 0, sizeof(rec));
  parser.onFrame(record, &rec);
  auto posllh = frame(0x01, 0x02, std::vector<uint8_t>(28, 0x55));
  posllh[10] ^= 0xFF;
  TEST_ASSERT_EQUAL_UINT32(0, parser.write(posllh.data(), posllh.size()));
  TEST_ASSERT_EQUAL_UINT32(0, rec.calls);
  TEST_ASSERT_EQUAL_UINT32(1, parser.checksumErrorCount());
}

static void test_handler_skips_oversized_frames(void) {
  UbxPvtParser parser;
  memset(&rec, 0, sizeof(rec));
  parser.onFrame(record, &rec);
  auto big = frame(0x01, 0x35, std::vector<uint8_t>(1500, 0x11)); // NAV-SAT beyond MaxFrameLen
  TEST_ASSERT_EQUAL_UINT32(0, parser.write(big.data(), big.size()));
  TEST_ASSERT_EQUAL_UINT32(0, rec.calls);
  TEST_ASSERT_EQUAL_UINT32(1, parser.skippedMessageCount());
}

static void test_handler_zero_length_frame(void) {
  UbxPvtParser parser;
  memset(&rec, 0, sizeof(rec));
  parser.onFrame(record, &rec);
  auto empty = frame(0x0A, 0x04, std::vector<uint8_t>());
  TEST_ASSERT_EQUAL_UINT32(0, parser.write(empty.data(), empty.size()));
  TEST_ASSERT_EQUAL_UINT32(1, rec.calls);
  TEST_ASSERT_EQUAL_UINT16(0, rec.len);
  TEST_ASSERT_EQUAL_UINT8(0x0A, rec.cls);
  TEST_ASSERT_EQUAL_UINT8(0x04, rec.id);
}

static void test_handler_can_be_unregistered(void) {
  UbxPvtParser parser;
  memset(&rec, 0, sizeof(rec));
  parser.onFrame(record, &rec);
  auto posllh = frame(0x01, 0x02, std::vector<uint8_t>(28, 0x55));
  parser.write(posllh.data(), posllh.size());
  TEST_ASSERT_EQUAL_UINT32(1, rec.calls);

  parser.onFrame(nullptr);
  parser.write(posllh.data(), posllh.size());
  TEST_ASSERT_EQUAL_UINT32(1, rec.calls); // unchanged
  TEST_ASSERT_EQUAL_UINT32(1, parser.skippedMessageCount());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_decode_valid_frame);
  RUN_TEST(test_decode_byte_by_byte);
  RUN_TEST(test_checksum_error_then_resync);
  RUN_TEST(test_other_ubx_messages_are_skipped);
  RUN_TEST(test_noise_nMEA_and_partial_frames);
  RUN_TEST(test_two_frames_back_to_back);
  RUN_TEST(test_short_v0_payload_accepted);
  RUN_TEST(test_oversized_pvt_is_skipped);
  RUN_TEST(test_no_fix_flags);
  RUN_TEST(test_reset);
  RUN_TEST(test_sync_count);
  RUN_TEST(test_fuzz_noise_recovery);
  RUN_TEST(test_sync_pair_inside_skipped_payload);
  RUN_TEST(test_handler_receives_pvt_and_other_frames);
  RUN_TEST(test_handler_context_pointer);
  RUN_TEST(test_handler_ignores_bad_checksum);
  RUN_TEST(test_handler_skips_oversized_frames);
  RUN_TEST(test_handler_zero_length_frame);
  RUN_TEST(test_handler_can_be_unregistered);
  return UNITY_END();
}
