#include <Arduino.h>
#include <UbxPvt.h>

// Demo / test sur cible : GPS M9N avec TX sur GPIO2. Lecture seule.
// Balaye les bauds courants, imprime la solution NAV-PVT et, toutes les 5 s,
// le decompte des types de trames recues (via le handler generique).

static HardwareSerial gpsSerial(1);
static UbxPvt gps(gpsSerial);

static const int8_t GPS_RX_PIN = 2;
static const int8_t GPS_TX_PIN = 3; // non câblé: la lib ne sert que de la lecture
static const uint32_t BAUD_CANDIDATES[] = {115200, 38400, 9600, 19200, 57600};

struct FrameStat {
  uint8_t cls;
  uint8_t id;
  uint32_t count;
};
static FrameStat frameStats[10];

// Diagnostics RFC (decodes dans le handler, pas dans la lib)
// NAV-SAT (u-blox 9, protocole 27+) : iTOW(0-3), version(4), numSvs(5), items de 12 octets
// a partir de 8 : gnssId, svId, cno(+2), elev(+3), azim(+4), prRes(+6), flags(+8).
// MON-HW (u-blox 9) : pinSel(0-3), pinBank(4-7), pinDir(8-11), pinVal(12-15), noisePerMS(16-17).
static uint8_t satInView = 0;    // NAV-SAT: nb de satellites decrits
static uint8_t satTracked = 0;   // dont le C/N0 > 0
static uint8_t satMeanCno = 0;   // C/N0 moyen des satellites suivis (dBHz)
static uint16_t satLen = 0;      // longueur brute de la trame NAV-SAT
static uint16_t noisePerMs = 0;  // MON-HW: niveau de bruit du coeur GNSS

static void onFrame(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t length, void* context) {
  (void)context;
  for (FrameStat& s : frameStats) {
    if (s.count == 0 || (s.cls == msgClass && s.id == msgId)) {
      s.cls = msgClass;
      s.id = msgId;
      s.count++;
      break;
    }
  }
  if (msgClass == 0x01 && msgId == 0x35 && length >= 8) { // NAV-SAT
    uint8_t numSvs = payload[5];
    satInView = numSvs;
    satLen = length;
    if (numSvs > 0 && length >= (uint16_t)(8u + 12u * numSvs)) {
      uint32_t sum = 0;
      uint8_t tracked = 0;
      for (uint8_t i = 0; i < numSvs; i++) {
        uint8_t cno = payload[8 + 12u * i + 2];
        if (cno > 0) {
          tracked++;
          sum += cno;
        }
      }
      satTracked = tracked;
      satMeanCno = tracked ? (uint8_t)((sum + tracked / 2) / tracked) : 0;
    }
  } else if (msgClass == 0x0A && msgId == 0x09 && length >= 18) { // MON-HW
    noisePerMs = (uint16_t)(payload[16] | (payload[17] << 8));
  }
}

static const char* frameName(uint8_t msgClass, uint8_t msgId) {
  switch ((msgClass << 8) | msgId) {
  case 0x0102: return "NAV-POSLLH";
  case 0x0103: return "NAV-STATUS";
  case 0x0104: return "NAV-DOP";
  case 0x0107: return "NAV-PVT";
  case 0x0112: return "NAV-VELNED";
  case 0x0121: return "NAV-TIMEUTC";
  case 0x0135: return "NAV-SAT";
  case 0x0213: return "RXM-SFRBX";
  case 0x0215: return "RXM-RAWX";
  case 0x0500: return "ACK-NAK";
  case 0x0501: return "ACK-ACK";
  case 0x0A04: return "MON-VER";
  case 0x0A06: return "MON-SYS";
  case 0x0A09: return "MON-HW";
  default: return nullptr;
  }
}

static bool detectBaud() {
  for (uint32_t baud : BAUD_CANDIDATES) {
    gps.begin(baud, GPS_RX_PIN, GPS_TX_PIN);
    uint32_t start = millis();
    while (millis() - start < 700) {
      gps.poll();
      if (gps.parser().syncCount() >= 4) {
        Serial.printf("[gps] trafic UBX detecte a %lu bauds\n", (unsigned long)baud);
        return true;
      }
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[demo] UbxPvt NAV-PVT (lecture seule)");

  if (!detectBaud()) {
    Serial.println("[gps] aucun trafic UBX, on reste a 115200");
  }
  gps.onFrame(onFrame);
}

void loop() {
  static uint32_t lastPrint = 0;
  static uint32_t lastStats = 0;

  if (gps.poll()) {
    const UbxNavPvt& pvt = gps.pvt();
    if (millis() - lastPrint >= 950) {
      lastPrint = millis();
      Serial.printf("[%02u/%02u/%04u %02u:%02u:%02u UTC] fixType=%u numSV=%u",
                    pvt.day, pvt.month, pvt.year, pvt.hour, pvt.minute, pvt.second,
                    pvt.fixType, pvt.numSV);
      if (pvt.hasFix()) {
        Serial.printf(" lat=%.7f lon=%.7f alt=%.1fm speed=%.2fm/s heading=%.1f hAcc=%.0fm",
                      pvt.longitudeDeg(), pvt.latitudeDeg(), pvt.altitudeMsl(),
                      pvt.groundSpeedMps(), pvt.headingDeg(), pvt.hAcc * 0.001);
      }
      Serial.println();
    }
  }

  if (millis() - lastStats >= 5000) {
    uint32_t elapsed = millis() - lastStats;
    lastStats = millis();
    Serial.printf("[stats] decoded=%lu ckErrors=%lu skipped=%lu\n",
                  (unsigned long)gps.parser().decodedCount(),
                  (unsigned long)gps.parser().checksumErrorCount(),
                  (unsigned long)gps.parser().skippedMessageCount());
    Serial.printf("[rf] bruit=%u | [sats] %u/%u avec signal (NAV-SAT len=%u), C/N0 moyen %u dBHz\n",
                  noisePerMs, satTracked, satInView, satLen, satMeanCno);
    Serial.print("[frames]");
    for (FrameStat& s : frameStats) {
      if (s.count == 0) continue;
      const char* name = frameName(s.cls, s.id);
      Serial.printf(" %s(%02X-%02X)=%lu/%.1fs", name ? name : "?", s.cls, s.id,
                    (unsigned long)s.count, s.count * 1000.0 / elapsed);
      s.count = 0;
    }
    Serial.println();
  }
}
