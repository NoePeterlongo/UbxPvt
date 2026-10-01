#include <Arduino.h>
#include <UbxPvt.h>

// Example: building a parser for another UBX message type on top of the
// generic onFrame() handler. Decodes UBX-NAV-SAT (per-satellite signal
// data) next to the library's built-in NAV-PVT decoding.
// Hardware: u-blox M9N on UART1, GPS TX wired to GPIO2 (ESP32-S3).

static HardwareSerial gpsSerial(1);
static UbxPvt         gps(gpsSerial);

static const int8_t   GPS_RX_PIN        = 2;
static const int8_t   GPS_TX_PIN        = 3;
static const uint32_t BAUD_CANDIDATES[] = {115200, 38400, 9600, 19200, 57600};

// UBX-NAV-SAT (0x01 0x35), u-blox 9 / protocol 27+.
// Payload: iTOW U4 (+0), version U1 (+4), numSvs U1 (+5), reserved U1[2] (+6),
// then one 12-byte item per satellite from offset 8:
//   gnssId U1 (+0), svId U1 (+1), cno U1 (+2), elev I1 (+3), azim I2 (+4),
//   prRes I2 (+6), flags X4 (+8)  -- flags bit 3: satellite used in solution.
// Note: on M8 receivers (protocol < 27) numSvs sits at offset 1 and there is
// no iTOW field.
static const uint8_t NAV_SAT_MAX_SVS = 40;

struct NavSatSat {
	uint8_t gnssId;
	uint8_t svId;
	uint8_t cno;  // dBHz
	int8_t  elev; // deg
	int16_t azim; // deg
	bool    used;
};

struct NavSat {
	uint32_t  iTOW   = 0;
	uint8_t   numSvs = 0;
	NavSatSat sats[NAV_SAT_MAX_SVS];
	bool      updated = false;

	bool decode(const uint8_t *payload, uint16_t length)
	{
		if (length < 8)
			return false;
		uint8_t n = payload[5];
		if (n == 0 || n > NAV_SAT_MAX_SVS ||
		    length < (uint16_t)(8u + 12u * n))
			return false;
		iTOW = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
		       ((uint32_t)payload[2] << 16) |
		       ((uint32_t)payload[3] << 24);
		numSvs = n;
		for (uint8_t i = 0; i < n; i++) {
			const uint8_t *p = payload + 8 + 12u * i;
			sats[i].gnssId   = p[0];
			sats[i].svId     = p[1];
			sats[i].cno      = p[2];
			sats[i].elev     = (int8_t)p[3];
			sats[i].azim     = (int16_t)(p[4] | (p[5] << 8));
			sats[i].used     = (p[8] & 0x08) != 0;
		}
		updated = true;
		return true;
	}
};

static NavSat navSat;

static void onFrame(uint8_t msgClass, uint8_t msgId, const uint8_t *payload,
                    uint16_t length, void *context)
{
	(void)context;
	if (msgClass == 0x01 && msgId == 0x35) // NAV-SAT
		navSat.decode(payload, length);
}

static const char *gnssName(uint8_t id)
{
	switch (id) {
	case 0: return "GPS";
	case 1: return "SBAS";
	case 2: return "GAL";
	case 3: return "BDS";
	case 4: return "IMES";
	case 5: return "QZSS";
	case 6: return "GLO";
	case 7: return "NavIC";
	default: return "?";
	}
}

static bool detectBaud()
{
	for (uint32_t baud : BAUD_CANDIDATES) {
		gpsSerial.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
		uint32_t start = millis();
		while (millis() - start < 700) {
			gps.poll();
			if (gps.parser().syncCount() >= 4) {
				Serial.printf(
				    "[gps] UBX traffic detected at %lu baud\n",
				    (unsigned long)baud);
				return true;
			}
		}
	}
	return false;
}

void setup()
{
	Serial.begin(115200);
	delay(500);
	Serial.println("[demo] UbxPvt NAV-SAT parser example");

	if (!detectBaud()) {
		Serial.println("[gps] no UBX traffic, staying at 115200");
	}
	gps.onFrame(onFrame);
}

void loop()
{
	static uint32_t lastPvtPrint = 0;
	static uint32_t lastSatPrint = 0;

	gps.poll();

	const UbxNavPvt &pvt = gps.pvt();
	if (millis() - lastPvtPrint >= 1000) {
		lastPvtPrint = millis();
		Serial.printf("[pvt] fixType=%u numSV=%u\n", pvt.fixType,
		              pvt.numSV);
	}

	if (navSat.updated && millis() - lastSatPrint >= 5000) {
		lastSatPrint   = millis();
		navSat.updated = false;
		Serial.printf("[nav-sat] iTOW=%lu sats=%u\n",
		              (unsigned long)navSat.iTOW, navSat.numSvs);
		for (uint8_t i = 0; i < navSat.numSvs; i++) {
			const NavSatSat &s = navSat.sats[i];
			if (s.cno == 0)
				continue;
			Serial.printf("  %-5s SV%02u  C/N0 %2u dBHz  elev %4d  "
			              "azim %5d%s\n",
			              gnssName(s.gnssId), s.svId, s.cno, s.elev,
			              s.azim, s.used ? "  *used" : "");
		}
	}
}
