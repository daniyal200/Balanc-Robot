//
// BalaC Plus — M5StickC Plus 1.1
// Self-balancing robot + VL53L0X ToF + 8BitDo Bluepad32
//
// FIXED: Implemented non-blocking ToF registry checks and removed high-latency
//        microsecond bus delays to preserve the strict 10ms PID balancing interval.
// EXTENDED: Added Autonomous Wandering State Machine when Bluetooth is not active.
// RESOLVED: Flipped autonomous move signs and guardrail logic to match physical forward direction.
//
// REQUIRES: Pololu VL53L0X library

#include <Arduino.h>
#include <M5StickCPlus.h>
#include <Wire.h>
#include <Bluepad32.h>
#include <VL53L0X.h>

// ── Pins ─────────────────────────────────────────────────────────────────────
#define LED           10
#define BUZZER_PIN    2       // M5StickC Plus onboard buzzer GPIO
#define MOTOR_SDA     0
#define MOTOR_SCL     26
#define MOTOR_ADDR    0x38
#define TOF_SDA       32
#define TOF_SCL       33
#define TOF_ADDR      0x29

// ── Tuning ───────────────────────────────────────────────────────────────────
#define N_CAL1                100
#define N_CAL2                100
#define LCDV_MID              60
#define OBSTACLE_THRESHOLD_MM 250
#define OBSTACLE_CLEAR_MM     340   // Hysteresis to prevent rapid stop/go chatter
#define BEEP_FREQ             4000
#define STICK_DEADZONE        40
#define STICK_MAX             512
#define MOVE_ALPHA            0.18f
#define SPIN_ALPHA            0.15f
#define TOF_INTERVAL_MS       50      // Checked smoothly outside critical windows
#define AUTO_FORWARD_MIN_RATE 0.40f   // Slow cruise when obstacle is near
#define AUTO_FORWARD_MAX_RATE 0.70f   // Nominal cruise speed
#define AUTO_TURN_RATE        24.0f   // Spin step magnitude during obstacle turn
#define AUTO_TURN_MIN_MS      650     // Minimum turn duration
#define AUTO_TURN_MAX_MS      1050    // Maximum turn duration
#define AUTO_SETTLE_MS        220     // Brief settle before returning to cruise
#define AUTO_YAW_KP           0.38f  // Proportional gain for heading-lock during straight cruise

// ── Autonomous States ────────────────────────────────────────────────────────
enum AutoState {
    AUTO_FORWARD,
    AUTO_TURNING,
    AUTO_SETTLING
};
AutoState currentAutoState = AUTO_FORWARD;
uint32_t stateTimer = 0;
uint32_t autoTurnDurationMs = AUTO_TURN_MIN_MS;
float    autoTurnDir = 1.0f;
float    autoTargetYaw = 0.0f;   // Locked heading for straight-line cruise

// ── Buzzer helpers — new core v3.x API, pin-based not channel-based ──────────
void buzzerOn(uint32_t freq) {
    ledcWriteTone(BUZZER_PIN, freq);
}
void buzzerOff() {
    ledcWriteTone(BUZZER_PIN, 0);
}

// ── Bluepad32 ────────────────────────────────────────────────────────────────
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool controllerConnected = false;

// ── ToF ──────────────────────────────────────────────────────────────────────
VL53L0X  tofSensor;
bool     tofAvailable     = false;
uint16_t tofDistance      = 8190;
bool     obstacleDetected = false;
uint32_t tofLastMs        = 0;

// ── Balance globals ───────────────────────────────────────────────────────────
boolean  standing      = false;
boolean  serialMonitor = true;
int16_t  counter       = 0;
uint32_t time0 = 0, time1 = 0;
int16_t  counterOverPwr = 0, maxOvp = 35;
float    power, powerR, powerL, yawPower;
float    varAng, varOmg, varSpd, varDst, varIang;
float    gyroXoffset, gyroYoffset, gyroZoffset, accXoffset;
float    gyroXdata, gyroYdata, gyroZdata, accXdata, accZdata;
float    aveAccZ = 0.0, aveAbsOmg = 0.0;
const float    cutoff   = 0.1;
const float    clk      = 0.01;
const uint32_t interval = (uint32_t)(clk * 1000);
const float    moveStep = 0.3f * clk;
float    Kang, Komg, KIang, Kyaw, Kdst, Kspd;
int16_t  maxPwr;
float    yawAngle   = 0.0;
float    moveTarget = 0.0, moveRate = 0.0;
int16_t  fbBalance = 0, motorDeadband = 0;
float    mechFactR, mechFactL;
bool     spinContinuous = false;
float    spinDest = 0, spinTarget = 0, spinFact = 1.0, spinStep = 0;
int16_t  ipowerL = 0, ipowerR = 0;
int16_t  motorLdir = 0, motorRdir = 0;
float    vBatt;
int16_t  punchPwr, punchPwr2, punchDur;
int16_t  punchCountL = 0, punchCountR = 0;
byte     demoMode = 0;
float    rawMoveRate = 0, rawSpinStep = 0;
float    ctrlMoveRate = 0, ctrlSpinStep = 0;

// ── Prototypes ────────────────────────────────────────────────────────────────
void  motorBusSelect();
void  tofBusSelect();
void  motorScan();
void  tofInit();
void  imuInit();
void  resetMotor();
void  resetPara();
void  resetVar();
void  calib1();
void  calib2();
void  calDelay(int n);
void  readGyro();
void  getGyro();
void  drive();
void  drvMotor(byte ch, int8_t sp);
void  drvMotorL(int16_t pwm);
void  drvMotorR(int16_t pwm);
void  setMode(bool inc);
void  checkButtonP();
void  dispBatVolt();
void  dispTof();
void  sendStatus();
void  checkObstacle();
void  processControllers();
void  processGamepad(ControllerPtr ctl);
float applyDeadzone(int16_t v, int16_t dz, int16_t mx);
void  onConnectedController(ControllerPtr ctl);
void  onDisconnectedController(ControllerPtr ctl);
void  updateAutonomousNavigation();

// ═══════════════════════════════════════════════════════════════════════════
//  BUS SWITCHING (Optimized for speed — removed heavy blocking delays)
// ═══════════════════════════════════════════════════════════════════════════
void motorBusSelect() {
    Wire.end();
    Wire.begin(MOTOR_SDA, MOTOR_SCL);
    Wire.setClock(400000);
}

void tofBusSelect() {
    Wire.end();
    Wire.begin(TOF_SDA, TOF_SCL);
    Wire.setClock(400000);
}

// ═══════════════════════════════════════════════════════════════════════════
//  MOTOR DRIVER
// ═══════════════════════════════════════════════════════════════════════════
void drvMotor(byte ch, int8_t sp) {
    motorBusSelect();
    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(ch);
    Wire.write(sp);
    Wire.endTransmission();
}
void drvMotorL(int16_t pwm) { drvMotor(0, (int8_t)constrain( pwm, -127, 127)); }
void drvMotorR(int16_t pwm) { drvMotor(1, (int8_t)constrain(-pwm, -127, 127)); }
void resetMotor()            { drvMotorL(0); drvMotorR(0); counterOverPwr = 0; }

void motorScan() {
    Serial.println("--- Motor bus scan (SDA=0 SCL=26) ---");
    motorBusSelect();
    bool found = false;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X%s\n", a, a == MOTOR_ADDR ? " <- STM32 OK" : "");
            found = true;
        }
        delay(2);
    }
    M5.Lcd.setTextColor(found ? GREEN : RED, BLACK);
    M5.Lcd.setCursor(0, 45);
    M5.Lcd.print(found ? "MTR:OK    " : "MTR:MISS!!");
    M5.Lcd.setTextColor(WHITE, BLACK);
    Serial.println("--------------------------------------");
}

// ═══════════════════════════════════════════════════════════════════════════
//  TOF INIT
// ═══════════════════════════════════════════════════════════════════════════
void tofInit() {
    Serial.println("--- ToF init (SDA=32 SCL=33) ---");
    tofBusSelect();
    delay(150);

    bool found = false;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X%s\n", a, a == TOF_ADDR ? " <- VL53L0X OK" : "");
            found = true;
        }
        delay(2);
    }

    M5.Lcd.setCursor(0, 65);
    M5.Lcd.setTextColor(YELLOW, BLACK);

    if (!found) {
        Serial.println("  Nothing found on ToF bus");
        M5.Lcd.print("TOF:MISS  ");
        M5.Lcd.setTextColor(WHITE, BLACK);
        tofAvailable = false;
        motorBusSelect();
        return;
    }

    tofSensor.setBus(&Wire);
    tofSensor.setTimeout(200);

    if (!tofSensor.init()) {
        Serial.println("ToF init() failed");
        M5.Lcd.print("TOF:FAIL  ");
        M5.Lcd.setTextColor(WHITE, BLACK);
        tofAvailable = false;
        motorBusSelect();
        return;
    }

    tofSensor.setSignalRateLimit(0.25);
    tofSensor.setMeasurementTimingBudget(33000);
    tofSensor.startContinuous(30);

    tofAvailable = true;
    tofLastMs    = millis();
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print("TOF:OK    ");
    M5.Lcd.setTextColor(WHITE, BLACK);

    motorBusSelect();
}

// ═══════════════════════════════════════════════════════════════════════════
//  BLUEPAD32
// ═══════════════════════════════════════════════════════════════════════════
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (!myControllers[i]) {
            myControllers[i] = ctl;
            controllerConnected = true;
            M5.Lcd.setCursor(5, 25);
            M5.Lcd.setTextColor(GREEN, BLACK);
            M5.Lcd.print("BT OK ");
            M5.Lcd.setTextColor(WHITE, BLACK);
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
        if (myControllers[i] == ctl) { myControllers[i] = nullptr; break; }
    controllerConnected = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
        if (myControllers[i]) { controllerConnected = true; break; }
    if (!controllerConnected) {
        rawMoveRate = rawSpinStep = 0;
        M5.Lcd.setCursor(5, 25);
        M5.Lcd.setTextColor(RED, BLACK);
        M5.Lcd.print("BT -- ");
        M5.Lcd.setTextColor(WHITE, BLACK);
    }
}

float applyDeadzone(int16_t v, int16_t dz, int16_t mx) {
    if (abs(v) < dz) return 0.0f;
    float n = (float)(abs(v) - dz) / (float)(mx - dz);
    return (v > 0) ? constrain(n, 0, 1) : -constrain(n, 0, 1);
}

void processGamepad(ControllerPtr ctl) {
    rawMoveRate = -applyDeadzone(ctl->axisY(), STICK_DEADZONE, STICK_MAX);
    if (ctl->a())      rawMoveRate =  1.0f;
    else if (ctl->y()) rawMoveRate = -1.0f;
    float turn = applyDeadzone(ctl->axisX(), STICK_DEADZONE, STICK_MAX);
    if (abs(ctl->axisRX()) > STICK_DEADZONE)
        turn = applyDeadzone(ctl->axisRX(), STICK_DEADZONE, STICK_MAX);
    rawSpinStep = turn * 50.0f * clk;
}

void processControllers() {
    for (auto c : myControllers)
        if (c && c->isConnected() && c->hasData() && c->isGamepad())
            processGamepad(c);
}

// ═══════════════════════════════════════════════════════════════════════════
//  IMU
// ═══════════════════════════════════════════════════════════════════════════
void imuInit() {
    M5.Imu.Init();
    M5.Imu.SetGyroFsr(M5.Imu.GFS_250DPS);
    M5.Imu.SetAccelFsr(M5.Imu.AFS_4G);
}

void readGyro() {
    float gX, gY, gZ, aX, aY, aZ;
    M5.Imu.getGyroData(&gX, &gY, &gZ);
    M5.Imu.getAccelData(&aX, &aY, &aZ);
    gyroYdata = gX; gyroZdata = -gY; gyroXdata = -gZ;
    accXdata  = aZ; accZdata  = aY;
}

void getGyro() {
    readGyro();
    varOmg    = gyroYdata - gyroYoffset;
    yawAngle += (gyroZdata - gyroZoffset) * clk;
    varAng   += (varOmg + ((accXdata - accXoffset) * 57.3 - varAng) * cutoff) * clk;
}

// ═══════════════════════════════════════════════════════════════════════════
//  CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════
void calDelay(int n) {
    for (int i = 0; i < n; i++) { getGyro(); delay(9); }
}

void calib1() {
    calDelay(30);
    digitalWrite(LED, LOW);
    calDelay(80);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(30, LCDV_MID);
    M5.Lcd.print("Cal-1...");
    gyroYoffset = 0;
    for (int i = 0; i < N_CAL1; i++) {
        readGyro();
        gyroYoffset += gyroYdata;
        delay(9);
    }
    gyroYoffset /= N_CAL1;
    M5.Lcd.fillScreen(BLACK);
    digitalWrite(LED, HIGH);
}

void calib2() {
    resetVar(); resetMotor();
    digitalWrite(LED, LOW);
    calDelay(80);
    M5.Lcd.setCursor(30, LCDV_MID);
    M5.Lcd.print("Cal-2...");
    accXoffset = gyroZoffset = 0;
    for (int i = 0; i < N_CAL2; i++) {
        readGyro();
        accXoffset  += accXdata;
        gyroZoffset += gyroZdata;
        delay(9);
    }
    accXoffset  /= N_CAL2;
    gyroZoffset /= N_CAL2;
    M5.Lcd.fillScreen(BLACK);
    digitalWrite(LED, HIGH);
}

// ═══════════════════════════════════════════════════════════════════════════
//  PID PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════
void resetPara() {
    Kang          = 37.0;
    Komg          = 0.84;
    KIang         = 800.0;
    Kyaw          = 4.0;
    Kdst          = 35.0;
    Kspd          = 1.2;
    mechFactL     = 0.45;
    mechFactR     = 0.45;
    punchPwr      = 20;
    punchDur      = 1;
    fbBalance     = 0;
    motorDeadband = 10;
    maxPwr        = 150;
    maxOvp        = 35;
    punchPwr2     = max(punchPwr, motorDeadband);
}

void resetVar() {
    power = moveTarget = moveRate = 0;
    spinContinuous = false;
    spinDest = spinTarget = spinStep = yawAngle = 0;
    varAng = varOmg = varDst = varSpd = varIang = 0;
    rawMoveRate = rawSpinStep = ctrlMoveRate = ctrlSpinStep = 0;
    currentAutoState = AUTO_FORWARD;
    autoTurnDurationMs = AUTO_TURN_MIN_MS;
    autoTurnDir = 1.0f;
    autoTargetYaw = 0.0f;
}

// ═══════════════════════════════════════════════════════════════════════════
//  DRIVE
// ═══════════════════════════════════════════════════════════════════════════
void drive() {
    spinFact = (abs(moveRate) > 0.1) ? constrain(-(powerR + powerL) / 10.0, -1.0, 1.0) : 1.0;

    if (spinContinuous) spinTarget += spinStep * spinFact;
    else {
        if (spinTarget < spinDest) spinTarget += spinStep;
        if (spinTarget > spinDest) spinTarget -= spinStep;
    }

    float dms = moveStep * fabsf(ctrlMoveRate);
    if (dms < moveStep * 0.05f) dms = moveStep * 0.05f;
    moveTarget += dms * (moveRate + (float)fbBalance / 100.0);

    varSpd  += power * clk;
    varDst  += Kdst * (varSpd * clk - moveTarget);
    varIang += KIang * varAng * clk;
    power    = varIang + varDst + Kspd*varSpd + Kang*varAng + Komg*varOmg;

    if (abs(power) > 1000.0) counterOverPwr++; else counterOverPwr = 0;
    if (counterOverPwr > maxOvp) return;

    power    = constrain(power, -maxPwr, maxPwr);
    yawPower = (yawAngle - spinTarget) * Kyaw;
    powerR   = power - yawPower;
    powerL   = power + yawPower;

    int16_t mdbn = -motorDeadband, pp2n = -punchPwr2;

    ipowerL = (int16_t)constrain(powerL * mechFactL, -maxPwr, maxPwr);
    if (ipowerL > 0) {
        punchCountL = (motorLdir == 1) ? constrain(++punchCountL, 0, 100) : 0;
        motorLdir = 1;
        drvMotorL(punchCountL < punchDur ? max(ipowerL, punchPwr2) : max(ipowerL, (int16_t)motorDeadband));
    } else if (ipowerL < 0) {
        punchCountL = (motorLdir == -1) ? constrain(++punchCountL, 0, 100) : 0;
        motorLdir = -1;
        drvMotorL(punchCountL < punchDur ? min(ipowerL, pp2n) : min(ipowerL, mdbn));
    } else { drvMotorL(0); motorLdir = 0; }

    ipowerR = (int16_t)constrain(powerR * mechFactR, -maxPwr, maxPwr);
    if (ipowerR > 0) {
        punchCountR = (motorRdir == 1) ? constrain(++punchCountR, 0, 100) : 0;
        motorRdir = 1;
        drvMotorR(punchCountR < punchDur ? max(ipowerR, punchPwr2) : max(ipowerR, (int16_t)motorDeadband));
    } else if (ipowerR < 0) {
        punchCountR = (motorRdir == -1) ? constrain(++punchCountR, 0, 100) : 0;
        motorRdir = -1;
        drvMotorR(punchCountR < punchDur ? min(ipowerR, pp2n) : min(ipowerR, mdbn));
    } else { drvMotorR(0); motorRdir = 0; }
}

// ═══════════════════════════════════════════════════════════════════════════
//  TOF OBSTACLE CHECK (Asynchronous & Ultra-fast Interrogation Fix)
// ═══════════════════════════════════════════════════════════════════════════
void checkObstacle() {
    if (!tofAvailable) return;
    uint32_t now = millis();
    if (now - tofLastMs < TOF_INTERVAL_MS) return;

    tofBusSelect();
    
    // CRITICAL FIX: Interrogate register 0x13 directly.
    // If the data is not fully compiled yet, skip the read sequence immediately 
    // to preserve processing time for the balance equations.
    if ((tofSensor.readReg(0x13) & 0x07) == 0) {
        motorBusSelect(); // Return cleanly
        return;
    }

    // Data is ready, reading here will be instant with zero blocking delay
    uint16_t d    = tofSensor.readRangeContinuousMillimeters();
    bool timedOut = tofSensor.timeoutOccurred();
    motorBusSelect();

    tofLastMs = now; // Only update timer when a hardware read successfully finishes

    if (timedOut || d >= 8000) {
        tofDistance      = 8190;
        obstacleDetected = false;
        buzzerOff();
        return;
    }

    tofDistance = d;

    if (!obstacleDetected && tofDistance < OBSTACLE_THRESHOLD_MM) {
        obstacleDetected = true;
        buzzerOn(BEEP_FREQ);
    } else if (obstacleDetected && tofDistance > OBSTACLE_CLEAR_MM) {
        obstacleDetected = false;
        buzzerOff();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  AUTONOMOUS STATE MACHINE
// ═══════════════════════════════════════════════════════════════════════════
void updateAutonomousNavigation() {
    // Only execute autonomous behavior if standing and NO controller is connected
    if (!standing || controllerConnected) return;

    uint32_t now = millis();

    // Speed tapers to AUTO_FORWARD_MIN_RATE as the obstacle enters the 1-metre warning zone
    float cruiseRate = AUTO_FORWARD_MAX_RATE;
    if (tofAvailable && tofDistance < 1000 && tofDistance >= OBSTACLE_THRESHOLD_MM) {
        float t = (float)(tofDistance - OBSTACLE_THRESHOLD_MM) /
                  (float)(1000 - OBSTACLE_THRESHOLD_MM);
        t = constrain(t, 0.0f, 1.0f);
        cruiseRate = AUTO_FORWARD_MIN_RATE +
                     t * (AUTO_FORWARD_MAX_RATE - AUTO_FORWARD_MIN_RATE);
    }

    switch (currentAutoState) {
        case AUTO_FORWARD:
            // Direct assignment — MOVE_ALPHA (0.18) provides a smooth ~100ms ramp on its own.
            // A second ramp layer was causing 600-800ms of near-zero command output.
            rawMoveRate = -cruiseRate;
            // Heading-lock: gently correct any yaw drift to keep the robot truly straight
            rawSpinStep = constrain(
                AUTO_YAW_KP * (autoTargetYaw - yawAngle) * clk, -0.008f, 0.008f);

            if (obstacleDetected) {
                currentAutoState = AUTO_TURNING;
                stateTimer = now;
                autoTurnDurationMs = random(AUTO_TURN_MIN_MS, AUTO_TURN_MAX_MS + 1);
                autoTurnDir = (random(0, 2) == 0) ? 1.0f : -1.0f;
            }
            break;

        case AUTO_TURNING:
            // Stop forward motion while pivoting away from the obstacle
            rawMoveRate = 0.0f;
            rawSpinStep = autoTurnDir * AUTO_TURN_RATE * clk;

            if ((now - stateTimer > autoTurnDurationMs) && !obstacleDetected) {
                currentAutoState = AUTO_SETTLING;
                stateTimer = now;
                rawSpinStep = 0.0f;
            }
            break;

        case AUTO_SETTLING:
            // Slow forward while spin decays to zero, then resume full cruise
            rawMoveRate = -AUTO_FORWARD_MIN_RATE;
            rawSpinStep = 0.0f;

            if (now - stateTimer > AUTO_SETTLE_MS) {
                autoTargetYaw = yawAngle;  // Lock onto new heading after the turn
                currentAutoState = AUTO_FORWARD;
            }
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  UI
// ═══════════════════════════════════════════════════════════════════════════
void dispBatVolt() {
    vBatt = M5.Axp.GetBatVoltage();
    M5.Lcd.setCursor(35, LCDV_MID);
    M5.Lcd.printf("%4.2fv ", vBatt);
}

void dispTof() {
    M5.Lcd.setCursor(0, 80);
    if (!tofAvailable) {
        M5.Lcd.setTextColor(DARKGREY, BLACK);
        M5.Lcd.print("TOF:N/A   ");
    } else if (tofDistance >= 8000) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print("TOF:----  ");
    } else {
        M5.Lcd.setTextColor(
            tofDistance < OBSTACLE_THRESHOLD_MM ? RED :
            tofDistance < 500                   ? YELLOW : GREEN, BLACK);
        M5.Lcd.printf("TOF:%4dmm", tofDistance);
    }
    M5.Lcd.setCursor(0, 100);
    if (obstacleDetected) {
        M5.Lcd.setTextColor(RED, BLACK);
        M5.Lcd.print("!!STOP!!! ");
    } else {
        M5.Lcd.setTextColor(BLACK, BLACK);
        M5.Lcd.print("          ");
    }
    M5.Lcd.setTextColor(WHITE, BLACK);
}

void sendStatus() {
    Serial.printf("stand=%d ang=%.2f pwr=%.0f mv=%.2f tof=%dmm obs=%d state=%d\n",
                  standing, varAng, power, moveRate, tofDistance, obstacleDetected, currentAutoState);
}

void checkButtonP() {
    byte p = M5.Axp.GetBtnPress();
    if (p == 2) calib1();
    else if (p == 1) setMode(true);
}

void setMode(bool inc) {
    if (inc) demoMode = ++demoMode % 2;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(20, 5);
    M5.Lcd.print(demoMode == 0 ? "Stand" : "Demo ");
    M5.Lcd.setCursor(5, 25);
    M5.Lcd.setTextColor(controllerConnected ? GREEN : YELLOW, BLACK);
    M5.Lcd.print(controllerConnected ? "BT OK " : "BT..  ");
    M5.Lcd.setTextColor(WHITE, BLACK);
}

void startDemo() {
    moveRate = 1.0;
    spinContinuous = true;
    spinStep = -40.0 * clk;
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    ledcAttach(BUZZER_PIN, BEEP_FREQ, 10);
    buzzerOff();

    M5.begin();
    Serial.begin(115200);
    delay(300);

    Wire.begin(MOTOR_SDA, MOTOR_SCL);
    Wire.setClock(400000);
    delay(100);

    imuInit();

    M5.Axp.ScreenBreath(11);
    M5.Lcd.setRotation(2);
    M5.Lcd.setTextFont(4);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE, BLACK);

    resetPara();
    resetVar();

    motorScan();
    delay(200);

    tofInit();
    delay(100);

    calib1();

    BP32.setup(&onConnectedController, &onDisconnectedController, true);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
    BP32.enableBLEService(false);

    setMode(false);
    time0 = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    BP32.update();
    processControllers();
    checkButtonP();
    
    // 1. Instantly read the IMU data to feed variables accurately.
    getGyro();

    // Handle autonomous navigation choices smoothly if no controller targets are active
    if (!controllerConnected) {
        updateAutonomousNavigation();
    }

    ctrlMoveRate += MOVE_ALPHA * (rawMoveRate - ctrlMoveRate);
    ctrlSpinStep += SPIN_ALPHA * (rawSpinStep - ctrlSpinStep);

    // Apply smoothed move/spin targets whenever the robot is standing
    if (standing) {
        moveRate = ctrlMoveRate;
        if (fabsf(ctrlSpinStep) > 0.0005f) {
            spinContinuous = true;
            spinStep = ctrlSpinStep;
        } else {
            spinContinuous = false;
            spinStep = 0;
        }
    }

    // FIXED: Guardrail flipped from > 0 to < 0 since negative moveRate is now forward motion.
    // This stops it from charging into objects.
    if (obstacleDetected && standing && moveRate < 0) {
        moveRate = 0;
    }

    if (!standing) {
        dispBatVolt();
        aveAbsOmg = aveAbsOmg * 0.9 + abs(varOmg) * 0.1;
        aveAccZ   = aveAccZ   * 0.9 + accZdata     * 0.1;
        M5.Lcd.setCursor(30, 130);
        M5.Lcd.printf("%5.2f  ", -aveAccZ);
        if (abs(aveAccZ) > 0.9 && aveAbsOmg < 1.5) {
            calib2();
            // Capture heading at balance-up so AUTO_FORWARD tracks straight from the start
            autoTargetYaw = yawAngle;
            if (demoMode == 1 && !controllerConnected) startDemo();
            standing = true;
        }
    } else {
        if (abs(varAng) > 30.0 || counterOverPwr > maxOvp) {
            resetMotor();
            resetVar();
            standing = false;
            setMode(false);
        } else {
            // 2. Perform the critical high-frequency balance driving adjustment
            drive();
        }
    }

    if (++counter >= 100) {
        counter = 0;
        dispBatVolt();
        dispTof();
        if (serialMonitor) sendStatus();
    }

    // 3. ToF checking operates here in non-blocking fashion
    checkObstacle();

    // 4. Time lock check maintains strict 10ms pacing
    do { time1 = millis(); } while (time1 - time0 < interval);
    time0 = time1;
}