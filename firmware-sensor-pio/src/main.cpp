/*
 * FINN Guidance — ESP32 Sensor Module Firmware
 *
 * Reads three sensor sources and sends them to the PC over USB serial:
 *   - GPS: NMEA passthrough from Quectel LC29H DA on UART2 (GPIO 16 RX / 17 TX)
 *   - WAS: 10kΩ pot on ADC1 channel 6 (GPIO 34), powered by GPIO 33 (3.3V HIGH)
 *   - IMU: BNO055 on I2C (GPIO 21 SDA / 22 SCL)
 *
 * Serial protocol (text-based, NMEA-style):
 *   GPS:  forwarded raw — $GPGGA,...  $GPVTG,...
 *   WAS:  $FINNWAS,<raw_adc>,<voltage_mv>*<checksum>\r\n
 *   IMU:  $FINNIMU,<roll>,<pitch>,<heading>,<cal_sys>,<cal_gyro>,<cal_accel>,<cal_mag>*<checksum>\r\n
 *   HB:   $FINNHB,<uptime_ms>*<checksum>\r\n
 *
 * Rates: WAS + IMU at ~20Hz, GPS passthrough at 1Hz, heartbeat every 2s.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <HardwareSerial.h>

// ── Pin assignments ──────────────────────────────────────────────────
#define WAS_PIN        34   // ADC input — pot wiper
#define WAS_POWER_PIN  33   // Output HIGH — pot 3.3V reference
#define GPS_RX_PIN     16   // UART2 RX — from ArduSimple TX
#define GPS_TX_PIN     17   // UART2 TX — to ArduSimple RX
#define BNO_SDA_PIN    21   // I2C SDA
#define BNO_SCL_PIN    22   // I2C SCL

// ── Timing ───────────────────────────────────────────────────────────
#define SENSOR_INTERVAL_MS    50    // 20Hz for WAS + IMU
#define HEARTBEAT_INTERVAL_MS 2000  // Every 2 seconds

// ── GPS UART ─────────────────────────────────────────────────────────
HardwareSerial gpsSerial(2);  // UART2

// ── BNO055 IMU ───────────────────────────────────────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool bnoAvailable = false;

// ── GPS line buffer ──────────────────────────────────────────────────
char gpsLineBuf[256];
int gpsLinePos = 0;

// ── Timing state ─────────────────────────────────────────────────────
unsigned long lastSensorMs = 0;
unsigned long lastHeartbeatMs = 0;

// ── NMEA checksum (XOR of all chars in body, between $ and *) ────────
uint8_t nmeaChecksum(const char* body) {
    uint8_t cs = 0;
    while (*body) {
        cs ^= (uint8_t)*body;
        body++;
    }
    return cs;
}

void setup() {
    // USB serial to PC
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        // Wait up to 2s for USB serial connection
    }
    Serial.println("FINN Sensor Module starting...");

    // WAS pot power — drive GPIO 33 HIGH to provide 3.3V reference
    pinMode(WAS_POWER_PIN, OUTPUT);
    digitalWrite(WAS_POWER_PIN, HIGH);
    Serial.println("GPIO 33 set HIGH — WAS pot powered (3.3V)");

    // ADC setup — GPIO 34 is input-only, no pinMode needed
    // analogReadResolution defaults to 12-bit (0–4095) on ESP32
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // Full 0–3.3V range
    Serial.println("ADC1 CH6 (GPIO 34) configured — WAS input");

    // I2C for BNO055
    Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN);
    Wire.setClock(400000);  // 400kHz
    Serial.println("I2C0 configured — SDA=21, SCL=22, 400kHz");

    // Initialise BNO055
    if (bno.begin()) {
        bno.setExtCrystalUse(true);
        bnoAvailable = true;
        Serial.println("BNO055 initialised in NDOF mode");
    } else {
        Serial.println("BNO055: no response on I2C");
        Serial.println("BNO055 init failed — IMU data will be zeros. Check wiring.");
    }

    // GPS UART2 — 115200 baud, RX on GPIO 16, TX on GPIO 17
    gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("UART2 configured — RX=16, TX=17, 115200 baud (GPS)");

    Serial.println("All peripherals initialised. Entering main loop.");
}

void loop() {
    unsigned long now = millis();

    // ── GPS passthrough: read UART2, forward complete NMEA lines ─────
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        if (c == '\n') {
            if (gpsLinePos > 0) {
                gpsLineBuf[gpsLinePos] = '\0';
                // Trim trailing \r
                if (gpsLinePos > 0 && gpsLineBuf[gpsLinePos - 1] == '\r') {
                    gpsLineBuf[gpsLinePos - 1] = '\0';
                }
                // Only forward NMEA sentences (starts with $)
                if (gpsLineBuf[0] == '$') {
                    Serial.println(gpsLineBuf);
                }
                gpsLinePos = 0;
            }
        } else if (gpsLinePos < (int)sizeof(gpsLineBuf) - 1) {
            gpsLineBuf[gpsLinePos++] = c;
        } else {
            // Buffer overflow — discard line
            gpsLinePos = 0;
        }
    }

    // ── Sensor reads at 20Hz ─────────────────────────────────────────
    if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
        lastSensorMs = now;

        // Read WAS (wheel angle sensor)
        int wasRaw = analogRead(WAS_PIN);
        int wasMv = (int)((uint32_t)wasRaw * 3300 / 4095);

        char wasBody[40];
        snprintf(wasBody, sizeof(wasBody), "FINNWAS,%d,%d", wasRaw, wasMv);
        uint8_t wasCs = nmeaChecksum(wasBody);
        Serial.printf("$%s*%02X\r\n", wasBody, wasCs);

        // Read IMU
        if (bnoAvailable) {
            sensors_event_t event;
            bno.getEvent(&event);

            // Get calibration status
            uint8_t sysCal, gyroCal, accelCal, magCal;
            bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);

            // event.orientation: x=heading, y=roll, z=pitch
            char imuBody[100];
            snprintf(imuBody, sizeof(imuBody),
                     "FINNIMU,%.1f,%.1f,%.1f,%d,%d,%d,%d",
                     event.orientation.y,   // roll
                     event.orientation.z,   // pitch
                     event.orientation.x,   // heading
                     sysCal, gyroCal, accelCal, magCal);
            uint8_t imuCs = nmeaChecksum(imuBody);
            Serial.printf("$%s*%02X\r\n", imuBody, imuCs);
        }
    }

    // ── Heartbeat every 2s ───────────────────────────────────────────
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatMs = now;

        char hbBody[30];
        snprintf(hbBody, sizeof(hbBody), "FINNHB,%lu", now);
        uint8_t hbCs = nmeaChecksum(hbBody);
        Serial.printf("$%s*%02X\r\n", hbBody, hbCs);
    }

    // Brief yield — GPS bytes buffered by hardware UART
    delay(1);
}
