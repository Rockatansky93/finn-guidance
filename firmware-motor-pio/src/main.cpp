/*
 * FINN Guidance — ESP32 Motor Controller + Inner Loop Firmware
 *
 * Decision #026: This ESP32 is now the sole microcontroller. It reads the WAS
 * (wheel angle sensor) locally and runs a closed-loop inner controller at
 * ~100Hz. The PC sends a desired steering angle (not PWM); the ESP32 compares
 * it against the actual WAS angle and drives the motor to match.
 *
 * Pinout:
 *   GPIO 25 — IBT-2 RPWM (steer right, PWM)
 *   GPIO 26 — IBT-2 LPWM (steer left, PWM)
 *   GPIO 27 — IBT-2 R_EN (right enable)
 *   GPIO 14 — IBT-2 L_EN (left enable)
 *   GPIO 34 — WAS pot wiper (ADC input, input-only)
 *   GPIO 33 — WAS pot VCC (3.3V reference, output HIGH)
 *
 * Serial protocol (text-based, NMEA-style):
 *   Receive:
 *     $FINNSTEER,<desired_angle_x100>*<checksum>\r\n
 *       desired_angle_x100: desired wheel angle x 100 (e.g. -523 = -5.23 deg)
 *       Positive = steer right. Sign is pre-inverted by PC if motor_invert is set.
 *
 *     $FINNCFG,WAS,<centre>,<left>,<right>*<checksum>\r\n
 *       Three-point WAS calibration. Stored in NVS. Values are raw ADC counts.
 *
 *     $FINNCFG,PID,<kp_x100>,<min_pwm>,<max_pwm>*<checksum>\r\n
 *       Inner loop tuning. kp_x100 = kp_angle x 100 (e.g. 1000 = 10.0 PWM/deg).
 *
 *     $FINNCFG,INVERT,<0|1>*<checksum>\r\n
 *       Motor direction invert flag. 1 = flip PWM sign after PID.
 *
 *   Send:
 *     $FINNMTR,<pwm>,<was_raw>,<angle_x100>,<enabled>,<uptime_ms>*<checksum>\r\n
 *       Status at 10Hz. angle_x100 = calibrated angle x 100.
 *
 *     $FINNACK,<param>,<status>*<checksum>\r\n
 *       Acknowledgement after $FINNCFG. status = "OK" or "ERR".
 *
 * Inner loop (runs at ~100Hz):
 *   1. Read WAS ADC -> piecewise-linear map to angle using NVS calibration
 *   2. angle_error = desired_angle - actual_angle
 *   3. raw_pwm = kp_angle * angle_error
 *   4. Sub-stall pulsing: accumulate sub-min_pwm effort, fire periodic pulses
 *   5. Apply motor_invert, clamp to max_pwm, drive IBT-2
 *
 * Safety:
 *   - Motor stops if no valid $FINNSTEER for 500ms (watchdog)
 *   - PWM clamped to +/-max_pwm
 *   - Enable lines driven LOW on startup and watchdog trip
 *   - desired_angle of 0 from PC -> ESP32 centres wheels (safe fallback)
 */

#include <Arduino.h>
#include <Preferences.h>

// ── Pin assignments ──────────────────────────────────────────────────
#define RPWM_PIN       25   // IBT-2 RPWM — steer right
#define LPWM_PIN       26   // IBT-2 LPWM — steer left
#define R_EN_PIN       27   // IBT-2 R_EN — right enable
#define L_EN_PIN       14   // IBT-2 L_EN — left enable
#define WAS_PIN        34   // ADC input — pot wiper (input-only GPIO)
#define WAS_POWER_PIN  33   // Output HIGH — pot 3.3V reference

// ── PWM configuration ────────────────────────────────────────────────
#define PWM_FREQ       20000  // 20kHz — above audible range
#define PWM_RESOLUTION 8      // 8-bit: 0-255
#define RPWM_CHANNEL   0      // LEDC channel for RPWM
#define LPWM_CHANNEL   1      // LEDC channel for LPWM

// ── Timing ───────────────────────────────────────────────────────────
#define WATCHDOG_TIMEOUT_MS    500   // Stop motor if no command for 500ms
#define STATUS_INTERVAL_MS     100   // 10Hz status reports
#define INNER_LOOP_INTERVAL_MS 10    // 100Hz inner loop

// ── WAS calibration constants ────────────────────────────────────────
#define MAX_STEER_ANGLE  45.0f  // Assumed max lock-to-lock half-angle (degrees)
#define WAS_SAMPLES      4      // ADC oversampling (average N reads for noise)

// ── NVS storage ──────────────────────────────────────────────────────
Preferences prefs;

// ── WAS calibration (from NVS) ───────────────────────────────────────
int16_t wasCentre = 1832;    // ADC count at straight-ahead
int16_t wasLeft   = 1617;    // ADC count at full left lock
int16_t wasRight  = 2031;    // ADC count at full right lock
bool wasCalibrated = false;  // True if all three values loaded from NVS

// ── Inner loop PID parameters (from NVS) ─────────────────────────────
float kpAngle   = 10.0f;    // PWM per degree of angle error
int16_t minPwm  = 100;      // Motor stall floor
int16_t maxPwm  = 180;      // Maximum PWM output
bool motorInvert = false;   // Flip PWM sign after PID

// ── Inner loop state ─────────────────────────────────────────────────
float desiredAngle = 0.0f;   // From PC via $FINNSTEER (degrees)
float actualAngle  = 0.0f;   // From local WAS (degrees)
int16_t wasRaw     = 0;      // Latest raw ADC reading
int16_t currentPwm = 0;      // PWM currently applied to motor
float pulseAccumulator = 0.0f;  // Sub-stall pulsing accumulator

// ── Control state ────────────────────────────────────────────────────
bool motorEnabled = false;
unsigned long lastCommandMs = 0;
unsigned long lastStatusMs  = 0;
unsigned long lastInnerLoopMs = 0;

// ── Serial line buffer ───────────────────────────────────────────────
char lineBuf[128];
int linePos = 0;

// ═════════════════════════════════════════════════════════════════════
// NMEA checksum (XOR of all chars in body, between $ and *)
// ═════════════════════════════════════════════════════════════════════
uint8_t nmeaChecksum(const char* body) {
    uint8_t cs = 0;
    while (*body) {
        cs ^= (uint8_t)*body;
        body++;
    }
    return cs;
}

// ═════════════════════════════════════════════════════════════════════
// Send a FINN sentence with auto-computed checksum
// ═════════════════════════════════════════════════════════════════════
void sendSentence(const char* body) {
    uint8_t cs = nmeaChecksum(body);
    Serial.printf("$%s*%02X\r\n", body, cs);
}

// ═════════════════════════════════════════════════════════════════════
// NVS: load calibration and PID parameters
// ═════════════════════════════════════════════════════════════════════
void loadNvsConfig() {
    prefs.begin("finn", true);  // read-only

    wasCentre = prefs.getShort("was_c", 1832);
    wasLeft   = prefs.getShort("was_l", 1617);
    wasRight  = prefs.getShort("was_r", 2031);

    // Consider calibrated if all three differ from each other
    wasCalibrated = (wasCentre != wasLeft) && (wasCentre != wasRight)
                    && (wasLeft != wasRight);

    kpAngle     = prefs.getFloat("kp", 10.0f);
    minPwm      = prefs.getShort("min_pwm", 100);
    maxPwm      = prefs.getShort("max_pwm", 180);
    motorInvert = prefs.getBool("invert", false);

    prefs.end();

    Serial.printf("NVS loaded — WAS C:%d L:%d R:%d cal:%s | Kp:%.1f min:%d max:%d inv:%d\r\n",
                  wasCentre, wasLeft, wasRight, wasCalibrated ? "YES" : "NO",
                  kpAngle, minPwm, maxPwm, motorInvert ? 1 : 0);
}

// ═════════════════════════════════════════════════════════════════════
// NVS: save WAS calibration
// ═════════════════════════════════════════════════════════════════════
void saveWasCal(int16_t c, int16_t l, int16_t r) {
    wasCentre = c;
    wasLeft   = l;
    wasRight  = r;
    wasCalibrated = (c != l) && (c != r) && (l != r);

    prefs.begin("finn", false);  // read-write
    prefs.putShort("was_c", c);
    prefs.putShort("was_l", l);
    prefs.putShort("was_r", r);
    prefs.end();

    Serial.printf("NVS saved WAS — C:%d L:%d R:%d\r\n", c, l, r);
}

// ═════════════════════════════════════════════════════════════════════
// NVS: save PID parameters
// ═════════════════════════════════════════════════════════════════════
void savePidConfig(float kp, int16_t minP, int16_t maxP) {
    kpAngle = kp;
    minPwm  = minP;
    maxPwm  = maxP;

    prefs.begin("finn", false);
    prefs.putFloat("kp", kp);
    prefs.putShort("min_pwm", minP);
    prefs.putShort("max_pwm", maxP);
    prefs.end();

    Serial.printf("NVS saved PID — Kp:%.1f min:%d max:%d\r\n", kp, minP, maxP);
}

// ═════════════════════════════════════════════════════════════════════
// NVS: save motor invert flag
// ═════════════════════════════════════════════════════════════════════
void saveMotorInvert(bool inv) {
    motorInvert = inv;

    prefs.begin("finn", false);
    prefs.putBool("invert", inv);
    prefs.end();

    Serial.printf("NVS saved INVERT — %d\r\n", inv ? 1 : 0);
}

// ═════════════════════════════════════════════════════════════════════
// WAS: read raw ADC with oversampling
// ═════════════════════════════════════════════════════════════════════
int16_t readWasRaw() {
    int32_t sum = 0;
    for (int i = 0; i < WAS_SAMPLES; i++) {
        sum += analogRead(WAS_PIN);
    }
    return (int16_t)(sum / WAS_SAMPLES);
}

// ═════════════════════════════════════════════════════════════════════
// WAS: convert raw ADC to calibrated angle (degrees)
//
// Piecewise linear mapping:
//   wasLeft   -> -MAX_STEER_ANGLE (full left)
//   wasCentre ->  0               (straight)
//   wasRight  -> +MAX_STEER_ANGLE (full right)
//
// This handles asymmetric steering geometry (left range != right range).
// ═════════════════════════════════════════════════════════════════════
float wasToAngle(int16_t raw) {
    if (!wasCalibrated) return 0.0f;

    if (raw <= wasCentre) {
        // Left half: map [wasLeft, wasCentre] -> [-MAX, 0]
        int16_t range = wasCentre - wasLeft;
        if (range == 0) return 0.0f;
        return -MAX_STEER_ANGLE * (float)(wasCentre - raw) / (float)range;
    } else {
        // Right half: map [wasCentre, wasRight] -> [0, +MAX]
        int16_t range = wasRight - wasCentre;
        if (range == 0) return 0.0f;
        return MAX_STEER_ANGLE * (float)(raw - wasCentre) / (float)range;
    }
}

// ═════════════════════════════════════════════════════════════════════
// Apply PWM value to IBT-2
//
// The Trimble EZ-Steer is direct-drive on the steering column — no gears.
// Hydraulic steering resistance stops the motor almost instantly when
// PWM is removed, so no explicit brake phase is needed on direction reversal.
// ═════════════════════════════════════════════════════════════════════
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

// ═════════════════════════════════════════════════════════════════════
// Kill motor — disable enable lines, zero PWM
// ═════════════════════════════════════════════════════════════════════
void killMotor() {
    currentPwm = 0;
    pulseAccumulator = 0.0f;
    ledcWrite(RPWM_CHANNEL, 0);
    ledcWrite(LPWM_CHANNEL, 0);
    digitalWrite(R_EN_PIN, LOW);
    digitalWrite(L_EN_PIN, LOW);
    motorEnabled = false;
}

// ═════════════════════════════════════════════════════════════════════
// Inner loop: desired angle vs actual WAS -> motor PWM
//
// This runs at ~100Hz (every 10ms). The PC sends a desired angle at
// ~10Hz; between PC updates, the inner loop keeps driving the motor
// toward the last received desired angle using local WAS feedback.
//
// Sub-stall pulsing: when the desired PWM is below the motor's stall
// threshold (minPwm), we accumulate the desired effort. When the
// accumulator reaches minPwm, one pulse is fired. This gives
// time-averaged torque below the stall floor.
// ═════════════════════════════════════════════════════════════════════
void runInnerLoop() {
    // Read WAS
    wasRaw = readWasRaw();
    actualAngle = wasToAngle(wasRaw);

    // If motor not enabled (watchdog tripped or never started), don't drive
    if (!motorEnabled) {
        currentPwm = 0;
        pulseAccumulator = 0.0f;
        return;
    }

    // Compute angle error
    float angleError = desiredAngle - actualAngle;

    // Proportional control
    float rawPwm = kpAngle * angleError;

    // Clamp to max
    if (rawPwm > (float)maxPwm) rawPwm = (float)maxPwm;
    if (rawPwm < -(float)maxPwm) rawPwm = -(float)maxPwm;

    float absDesired = fabsf(rawPwm);
    int16_t output;

    if (absDesired >= (float)minPwm) {
        // Above stall floor — direct drive with stall compensation
        pulseAccumulator = 0.0f;
        output = (int16_t)roundf(rawPwm);
        // Ensure at least minPwm magnitude
        if (output > 0 && output < minPwm) output = minPwm;
        if (output < 0 && output > -minPwm) output = -minPwm;
    } else if (absDesired < 1.0f) {
        // Negligible effort — don't accumulate noise
        pulseAccumulator = 0.0f;
        output = 0;
    } else {
        // Sub-stall zone: accumulate desired effort
        pulseAccumulator += rawPwm;

        if (fabsf(pulseAccumulator) >= (float)minPwm) {
            // Fire one pulse
            int16_t pulseDir = (pulseAccumulator > 0) ? 1 : -1;
            pulseAccumulator -= pulseDir * (float)minPwm;
            output = pulseDir * minPwm;
        } else {
            output = 0;
        }
    }

    // Apply motor direction invert
    if (motorInvert) output = -output;

    // Final clamp
    if (output > maxPwm) output = maxPwm;
    if (output < -maxPwm) output = -maxPwm;

    currentPwm = output;
    applyPwm(currentPwm);
}

// ═════════════════════════════════════════════════════════════════════
// Parse body of a validated FINN sentence (after checksum verification).
// Returns true if the sentence was recognised and handled.
// ═════════════════════════════════════════════════════════════════════
bool handleSentence(const char* body, unsigned long now) {

    // $FINNSTEER,<desired_angle_x100>
    if (strncmp(body, "FINNSTEER,", 10) == 0) {
        int16_t angleX100 = (int16_t)atoi(body + 10);
        desiredAngle = (float)angleX100 / 100.0f;
        lastCommandMs = now;

        if (!motorEnabled) {
            digitalWrite(R_EN_PIN, HIGH);
            digitalWrite(L_EN_PIN, HIGH);
            motorEnabled = true;
            pulseAccumulator = 0.0f;
            Serial.println("Motor ENABLED (first steer command)");
        }
        return true;
    }

    // $FINNCFG,WAS,<centre>,<left>,<right>
    if (strncmp(body, "FINNCFG,WAS,", 12) == 0) {
        int c, l, r;
        if (sscanf(body + 12, "%d,%d,%d", &c, &l, &r) == 3) {
            saveWasCal((int16_t)c, (int16_t)l, (int16_t)r);
            sendSentence("FINNACK,WAS,OK");
        } else {
            sendSentence("FINNACK,WAS,ERR");
        }
        return true;
    }

    // $FINNCFG,PID,<kp_x100>,<min_pwm>,<max_pwm>
    if (strncmp(body, "FINNCFG,PID,", 12) == 0) {
        int kpX100, minP, maxP;
        if (sscanf(body + 12, "%d,%d,%d", &kpX100, &minP, &maxP) == 3) {
            savePidConfig((float)kpX100 / 100.0f, (int16_t)minP, (int16_t)maxP);
            sendSentence("FINNACK,PID,OK");
        } else {
            sendSentence("FINNACK,PID,ERR");
        }
        return true;
    }

    // $FINNCFG,INVERT,<0|1>
    if (strncmp(body, "FINNCFG,INVERT,", 15) == 0) {
        int inv = atoi(body + 15);
        saveMotorInvert(inv != 0);
        sendSentence("FINNACK,INVERT,OK");
        return true;
    }

    return false;  // Unrecognised sentence
}

// ═════════════════════════════════════════════════════════════════════
// Parse a complete serial line. Validates $ prefix, * separator,
// and NMEA checksum before dispatching to handleSentence().
// ═════════════════════════════════════════════════════════════════════
void parseLine(const char* line, unsigned long now) {
    // Must start with dollar sign
    if (line[0] != 0x24) return;
    const char* bodyStart = line + 1;

    // Find the * separator
    const char* star = strchr(bodyStart, '*');
    if (!star) return;

    // Extract body
    int bodyLen = star - bodyStart;
    if (bodyLen <= 0 || bodyLen > 80) return;

    char body[84];
    strncpy(body, bodyStart, bodyLen);
    body[bodyLen] = '\0';

    // Verify checksum
    unsigned long expected = strtoul(star + 1, NULL, 16);
    uint8_t actual = nmeaChecksum(body);
    if ((uint8_t)expected != actual) {
        return;  // Silent discard — don't spam serial with checksum errors
    }

    handleSentence(body, now);
}

// ═════════════════════════════════════════════════════════════════════
// setup()
// ═════════════════════════════════════════════════════════════════════
void setup() {
    // USB serial
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        // Wait up to 2s for USB serial connection
    }
    Serial.println("FINN Motor Controller v2 starting (Decision #026)...");

    // WAS pot power — drive GPIO 33 HIGH to provide 3.3V reference
    pinMode(WAS_POWER_PIN, OUTPUT);
    digitalWrite(WAS_POWER_PIN, HIGH);
    Serial.println("GPIO 33 set HIGH — WAS pot powered (3.3V)");

    // ADC setup — GPIO 34 is input-only, no pinMode needed
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // Full 0-3.3V range
    Serial.println("ADC1 CH6 (GPIO 34) configured — WAS input");

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

    // Load calibration and PID params from NVS
    loadNvsConfig();

    Serial.println("All peripherals initialised. Entering main loop.");

    unsigned long now = millis();
    lastCommandMs    = now;
    lastStatusMs     = now;
    lastInnerLoopMs  = now;
}

// ═════════════════════════════════════════════════════════════════════
// loop()
// ═════════════════════════════════════════════════════════════════════
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

                parseLine(lineBuf, now);
                linePos = 0;
            }
        } else if (c != '\r' && linePos < (int)sizeof(lineBuf) - 1) {
            lineBuf[linePos++] = c;
        }
    }

    // ── Inner loop at ~100Hz ─────────────────────────────────────────
    if (now - lastInnerLoopMs >= INNER_LOOP_INTERVAL_MS) {
        lastInnerLoopMs = now;
        runInnerLoop();
    }

    // ── Watchdog: stop motor if no command for 500ms ─────────────────
    if (motorEnabled && (now - lastCommandMs > WATCHDOG_TIMEOUT_MS)) {
        Serial.printf("WATCHDOG: no command for %dms — motor STOPPED\r\n",
                      WATCHDOG_TIMEOUT_MS);
        killMotor();
    }

    // ── Status report at 10Hz ────────────────────────────────────────
    // Reports: PWM, raw WAS ADC, calibrated angle x 100, enabled, uptime
    if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
        lastStatusMs = now;

        int16_t angleX100 = (int16_t)roundf(actualAngle * 100.0f);
        int enabledFlag = motorEnabled ? 1 : 0;

        char body[80];
        snprintf(body, sizeof(body), "FINNMTR,%d,%d,%d,%d,%lu",
                 currentPwm, wasRaw, angleX100, enabledFlag, now);
        sendSentence(body);
    }

    // Brief yield — keep loop responsive without busy-spinning.
    // The inner loop timer handles 100Hz scheduling; this just prevents
    // starving the watchdog/WiFi tasks on the ESP32's RTOS.
    delay(1);
}
