/*
 * FINN Guidance — ESP32 Motor Controller Firmware
 *
 * Receives steer commands from the PC over USB serial and drives the IBT-2
 * H-bridge to control the steering motor.
 *
 * Pinout:
 *   GPIO 25 — IBT-2 RPWM (steer right, PWM)
 *   GPIO 26 — IBT-2 LPWM (steer left, PWM)
 *   GPIO 27 — IBT-2 R_EN (right enable)
 *   GPIO 14 — IBT-2 L_EN (left enable)
 *
 * Serial protocol (text-based, NMEA-style):
 *   Receive: $FINNSTEER,<pwm_value>*<checksum>\r\n
 *            pwm_value: -255 to 255 (negative=left, positive=right)
 *   Send:    $FINNMTR,<current_pwm>,<enabled>,<uptime_ms>*<checksum>\r\n  (status at 5Hz)
 *
 * Safety:
 *   - Motor stops if no valid command received within 500ms (watchdog)
 *   - PWM clamped to ±255
 *   - Enable lines driven LOW (motor off) on startup and watchdog trip
 */

#include <Arduino.h>

// ── Pin assignments ──────────────────────────────────────────────────
#define RPWM_PIN  25   // IBT-2 RPWM — steer right
#define LPWM_PIN  26   // IBT-2 LPWM — steer left
#define R_EN_PIN  27   // IBT-2 R_EN — right enable
#define L_EN_PIN  14   // IBT-2 L_EN — left enable

// ── PWM configuration ────────────────────────────────────────────────
#define PWM_FREQ       20000  // 20kHz — above audible range
#define PWM_RESOLUTION 8      // 8-bit: 0–255
#define RPWM_CHANNEL   0      // LEDC channel for RPWM
#define LPWM_CHANNEL   1      // LEDC channel for LPWM

// ── Timing ───────────────────────────────────────────────────────────
#define WATCHDOG_TIMEOUT_MS  500   // Stop motor if no command for 500ms
#define STATUS_INTERVAL_MS   200   // 5Hz status reports

// ── State ────────────────────────────────────────────────────────────
int16_t currentPwm = 0;
bool motorEnabled = false;
unsigned long lastCommandMs = 0;
unsigned long lastStatusMs = 0;

// ── Serial line buffer ───────────────────────────────────────────────
char lineBuf[128];
int linePos = 0;

// ── NMEA checksum (XOR of all chars in body, between $ and *) ────────
uint8_t nmeaChecksum(const char* body) {
    uint8_t cs = 0;
    while (*body) {
        cs ^= (uint8_t)*body;
        body++;
    }
    return cs;
}

// ── Apply PWM value to IBT-2 ─────────────────────────────────────────
void applyPwm(int16_t pwm) {
    uint8_t duty = (uint8_t)min(abs(pwm), 255);

    if (pwm > 0) {
        // Steer right: RPWM active, LPWM off
        ledcWrite(LPWM_CHANNEL, 0);
        ledcWrite(RPWM_CHANNEL, duty);
    } else if (pwm < 0) {
        // Steer left: LPWM active, RPWM off
        ledcWrite(RPWM_CHANNEL, 0);
        ledcWrite(LPWM_CHANNEL, duty);
    } else {
        // Stop: both off
        ledcWrite(RPWM_CHANNEL, 0);
        ledcWrite(LPWM_CHANNEL, 0);
    }
}

// ── Parse $FINNSTEER,<pwm>*<checksum> ────────────────────────────────
// Returns true and sets *pwmOut if valid, false otherwise.
bool parseSteerCommand(const char* line, int16_t* pwmOut) {
    // Must start with $
    if (line[0] != '$') return false;
    line++;  // skip $

    // Find the * separator
    const char* star = strchr(line, '*');
    if (!star) return false;

    // Extract body (between $ and *)
    int bodyLen = star - line;
    if (bodyLen <= 0 || bodyLen > 60) return false;

    char body[64];
    strncpy(body, line, bodyLen);
    body[bodyLen] = '\0';

    // Verify checksum
    const char* csStr = star + 1;
    unsigned long expected = strtoul(csStr, NULL, 16);
    uint8_t actual = nmeaChecksum(body);
    if ((uint8_t)expected != actual) {
        Serial.printf("Checksum mismatch: expected %02X, got %02X for '%s'\r\n",
                      (uint8_t)expected, actual, body);
        return false;
    }

    // Must start with "FINNSTEER,"
    if (strncmp(body, "FINNSTEER,", 10) != 0) return false;

    // Parse PWM value
    int16_t pwm = (int16_t)atoi(body + 10);

    // Clamp to valid range
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;

    *pwmOut = pwm;
    return true;
}

void setup() {
    // USB serial
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        // Wait up to 2s for USB serial connection
    }
    Serial.println("FINN Motor Controller starting...");

    // IBT-2 enable lines — start with motor disabled
    pinMode(R_EN_PIN, OUTPUT);
    pinMode(L_EN_PIN, OUTPUT);
    digitalWrite(R_EN_PIN, LOW);
    digitalWrite(L_EN_PIN, LOW);
    Serial.println("IBT-2 enable lines LOW (motor disabled)");

    // PWM setup using LEDC
    ledcSetup(RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(RPWM_PIN, RPWM_CHANNEL);
    ledcAttachPin(LPWM_PIN, LPWM_CHANNEL);
    ledcWrite(RPWM_CHANNEL, 0);
    ledcWrite(LPWM_CHANNEL, 0);
    Serial.println("PWM configured — RPWM=GPIO25, LPWM=GPIO26, 20kHz 8-bit");

    Serial.println("All peripherals initialised. Entering main loop.");

    lastCommandMs = millis();
}

void loop() {
    unsigned long now = millis();

    // ── Read serial input, parse complete lines ──────────────────────
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            if (linePos > 0) {
                lineBuf[linePos] = '\0';

                // Trim trailing \r
                if (linePos > 0 && lineBuf[linePos - 1] == '\r') {
                    lineBuf[linePos - 1] = '\0';
                }

                int16_t pwm;
                if (parseSteerCommand(lineBuf, &pwm)) {
                    currentPwm = pwm;
                    lastCommandMs = now;

                    if (!motorEnabled) {
                        digitalWrite(R_EN_PIN, HIGH);
                        digitalWrite(L_EN_PIN, HIGH);
                        motorEnabled = true;
                        Serial.println("Motor ENABLED (first command received)");
                    }

                    applyPwm(currentPwm);
                }
                linePos = 0;
            }
        } else if (c != '\r' && linePos < (int)sizeof(lineBuf) - 1) {
            lineBuf[linePos++] = c;
        }
    }

    // ── Watchdog: stop motor if no command for 500ms ─────────────────
    if (motorEnabled && (now - lastCommandMs > WATCHDOG_TIMEOUT_MS)) {
        Serial.printf("WATCHDOG: no command for %dms — motor STOPPED\r\n", WATCHDOG_TIMEOUT_MS);
        currentPwm = 0;
        ledcWrite(RPWM_CHANNEL, 0);
        ledcWrite(LPWM_CHANNEL, 0);
        digitalWrite(R_EN_PIN, LOW);
        digitalWrite(L_EN_PIN, LOW);
        motorEnabled = false;
    }

    // ── Status report at 5Hz ─────────────────────────────────────────
    if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
        lastStatusMs = now;
        int enabledFlag = motorEnabled ? 1 : 0;

        char body[50];
        snprintf(body, sizeof(body), "FINNMTR,%d,%d,%lu", currentPwm, enabledFlag, now);
        uint8_t cs = nmeaChecksum(body);
        Serial.printf("$%s*%02X\r\n", body, cs);
    }

    delay(1);
}
