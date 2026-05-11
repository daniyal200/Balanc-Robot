//
// BalaC Plus balancing robot with 8BitDo Bluetooth Controller
// Original balancing code by Kiraku Labo
// Bluepad32 integration for 8BitDo Ultimate C Bluetooth controller
//
// Calibration procedure:
//   1. Lay the robot flat, and power on.
//   2. Wait until Cal-1 (Pitch Gyro calibration) completes.
//   3. Hold still the robot upright in balance until Cal-2
//      (Accel & Yaw Gyro cal) completes.
//
// Controller mapping:
//   A button:           move forward
//   Y button:           move backward
//   Left stick X-axis:  turn left / right
//
// NOTE: Bluepad32 has a built-in interactive console.
//       It is incompatible with Arduino "Serial".
//       Use "Console" class instead of "Serial" for debug output.

#include <Arduino.h>
#include <M5StickCPlus.h>
#include <Wire.h>
#include <Bluepad32.h>
#include <VL53L0X.h>

// ── Pin & constant definitions ──────────────────────────────────────────────
#define LED        10       // Built-in LED on M5StickC Plus (GPIO10)
#define N_CAL1     100      // Number of samples for Cal-1 (pitch gyro)
#define N_CAL2     100      // Number of samples for Cal-2 (accel + yaw gyro)
#define LCDV_MID   60

// ── ToF obstacle detection ──────────────────────────────────────────────────
#define TOF_SDA               32     // Grove port SDA
#define TOF_SCL               33     // Grove port SCL
#define OBSTACLE_THRESHOLD_MM 200    // Stop & beep if closer than 200mm (20cm)
#define BEEP_FREQ             4000   // Buzzer frequency in Hz

// ── Controller deadzone ─────────────────────────────────────────────────────
#define STICK_DEADZONE  50  // Ignore stick values within ±50 (out of ±512)
#define STICK_MAX      512  // Max stick axis value

// ── Bluepad32 controller ────────────────────────────────────────────────────
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool controllerConnected = false;

// ── ToF sensor ──────────────────────────────────────────────────────────────
VL53L0X tofSensor;
bool tofAvailable       = false;
uint16_t tofDistance    = 8190;   // max = no obstacle
bool obstacleDetected   = false;

// ── Global variables (balancing) ────────────────────────────────────────────
boolean serialMonitor = true;
boolean standing      = false;
int16_t counter       = 0;
uint32_t time0 = 0, time1 = 0;
int16_t counterOverPwr = 0, maxOvp = 20;
float power, powerR, powerL, yawPower;
float varAng, varOmg, varSpd, varDst, varIang;
float gyroXoffset, gyroYoffset, gyroZoffset, accXoffset;
float gyroXdata, gyroYdata, gyroZdata, accXdata, accZdata;
float aveAccX = 0.0, aveAccZ = 0.0, aveAbsOmg = 0.0;
float cutoff            = 0.1;                     // ~= 2 * pi * f (Hz)
const float clk         = 0.01;                    // in sec
const uint32_t interval = (uint32_t)(clk * 1000);  // in msec
float Kang, Komg, KIang, Kyaw, Kdst, Kspd;
int16_t maxPwr;
float yawAngle = 0.0;
float moveDestination, moveTarget;
float moveRate        = 0.0;
const float moveStep  = 0.2 * clk;
int16_t fbBalance     = 0;
int16_t motorDeadband = 0;
float mechFactR, mechFactL;
int8_t motorRDir = 0, motorLDir = 0;
bool spinContinuous = false;
float spinDest, spinTarget, spinFact = 1.0;
float spinStep  = 0.0;  // deg per 10msec
int16_t ipowerL = 0, ipowerR = 0;
int16_t motorLdir = 0, motorRdir = 0;  // 0:stop 1:+ -1:-
float vBatt, voltAve             = 3.7;
int16_t punchPwr, punchPwr2, punchDur, punchCountL = 0, punchCountR = 0;
byte demoMode = 0;

// ── Controller input state ──────────────────────────────────────────────────
float ctrlMoveRate = 0.0;   // From left stick Y
float ctrlSpinStep = 0.0;   // From left stick X
bool  ctrlModeToggle = false;

// ── Function prototypes ─────────────────────────────────────────────────────
void imuInit();
void resetMotor();
void resetPara();
void resetVar();
void calib1();
void drvMotor(byte ch, int8_t sp);
void drvMotorL(int16_t pwm);
void drvMotorR(int16_t pwm);
void setMode(bool inc);
void dispBatVolt();
void getGyro();
void checkButtonP();
void calib2();
void startDemo();
void drive();
void calDelay(int n);
void sendStatus();
void readGyro();
void processControllers();
void processGamepad(ControllerPtr ctl);
void checkObstacle();

// ═══════════════════════════════════════════════════════════════════════════
//  BLUEPAD32 CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                           ctl->getModelName(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            controllerConnected = true;

            // Show on LCD
            M5.Lcd.setCursor(5, 25);
            M5.Lcd.setTextColor(GREEN, BLACK);
            M5.Lcd.print("BT OK ");
            M5.Lcd.setTextColor(WHITE, BLACK);
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but no empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }
    // Check if any controller is still connected
    controllerConnected = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr) {
            controllerConnected = true;
            break;
        }
    }
    if (!controllerConnected) {
        // Reset controller inputs when disconnected
        ctrlMoveRate = 0.0;
        ctrlSpinStep = 0.0;

        M5.Lcd.setCursor(5, 25);
        M5.Lcd.setTextColor(RED, BLACK);
        M5.Lcd.print("BT -- ");
        M5.Lcd.setTextColor(WHITE, BLACK);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  CONTROLLER INPUT PROCESSING
// ═══════════════════════════════════════════════════════════════════════════
float applyDeadzone(int16_t value, int16_t deadzone, int16_t maxVal) {
    if (abs(value) < deadzone) return 0.0;
    // Map from deadzone..max to 0..1.0
    float normalized = (float)(abs(value) - deadzone) / (float)(maxVal - deadzone);
    normalized = constrain(normalized, 0.0f, 1.0f);
    return (value > 0) ? normalized : -normalized;
}

void processGamepad(ControllerPtr ctl) {
    // 1. Left stick Y-axis → forward/backward movement
    // negative=up (forward on stick), positive=down (backward on stick)
    int16_t stickY = ctl->axisY();
    ctrlMoveRate = -applyDeadzone(stickY, STICK_DEADZONE, STICK_MAX);

    // 2. Buttons override: A/Y buttons
    if (ctl->a()) {
        ctrlMoveRate = 1.0;   // Forward on A (User requested earlier)
    } else if (ctl->y()) {
        ctrlMoveRate = -1.0;  // Backward on Y
    }

    // 3. Left stick X-axis → turning
    int16_t stickX = ctl->axisX();
    float turnInput = applyDeadzone(stickX, STICK_DEADZONE, STICK_MAX);
    // You can also use the Right stick for turning instead if preferred
    if (abs(ctl->axisRX()) > STICK_DEADZONE) {
        turnInput = applyDeadzone(ctl->axisRX(), STICK_DEADZONE, STICK_MAX);
    }
    
    ctrlSpinStep = turnInput * 40.0 * clk;  // Scale similar to demo spin
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    M5.begin();
    Wire.begin(0, 26);  // SDA = GPIO0, SCL = GPIO26 (to STM32 motor controller)
    imuInit();
    M5.Axp.ScreenBreath(11);
    M5.Lcd.setRotation(2);
    M5.Lcd.setTextFont(4);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
    resetMotor();
    resetPara();
    resetVar();

    // ── Initialize ToF sensor on Grove port (Wire1) ──
    Wire1.begin(TOF_SDA, TOF_SCL);
    tofSensor.setBus(&Wire1);
    tofSensor.setTimeout(500);
    if (tofSensor.init()) {
        tofSensor.startContinuous(50);  // New measurement every 50ms
        tofAvailable = true;
        Console.println("ToF sensor initialized OK");
    } else {
        Console.println("WARNING: ToF sensor not found!");
    }

    calib1();

    // ── Initialize Bluepad32 ──
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n",
                   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController, true);
    BP32.forgetBluetoothKeys();  // Fresh pairing each boot
    BP32.enableVirtualDevice(false);
    BP32.enableBLEService(false);

    // Show scanning status
    M5.Lcd.setCursor(5, 25);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("BT.. ");
    M5.Lcd.setTextColor(WHITE, BLACK);

    setMode(false);
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    // ── Update controller inputs ──
    BP32.update();
    processControllers();

    checkButtonP();
    getGyro();
    checkObstacle();

    // Apply controller input to movement
    if (controllerConnected && standing) {
        moveRate = ctrlMoveRate;
        spinContinuous = (abs(ctrlSpinStep) > 0.001);
        if (spinContinuous) {
            spinStep = ctrlSpinStep;
        } else {
            spinStep = 0.0;
        }
    }

    // Override: block forward movement if obstacle detected
    if (obstacleDetected && standing && moveRate > 0) {
        moveRate = 0;
    }

    if (!standing) {
        dispBatVolt();
        aveAbsOmg = aveAbsOmg * 0.9 + abs(varOmg) * 0.1;
        aveAccZ   = aveAccZ * 0.9 + accZdata * 0.1;
        M5.Lcd.setCursor(30, 130);
        M5.Lcd.printf("%5.2f   ", -aveAccZ);
        if (abs(aveAccZ) > 0.9 && aveAbsOmg < 1.5) {
            calib2();
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
            drive();
        }
    }
    counter += 1;
    if (counter >= 100) {
        counter = 0;
        dispBatVolt();
        if (serialMonitor) sendStatus();
    }
    do time1 = millis();
    while (time1 - time0 < interval);
    time0 = time1;
}

// ═══════════════════════════════════════════════════════════════════════════
//  IMU INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════
void imuInit() {
    M5.Imu.Init();
    M5.Imu.SetGyroFsr(M5.Imu.GFS_250DPS);
    M5.Imu.SetAccelFsr(M5.Imu.AFS_4G);
    Console.println("MPU6886 found");
}

// ═══════════════════════════════════════════════════════════════════════════
//  CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════
void calib1() {
    calDelay(30);
    digitalWrite(LED, LOW);
    calDelay(80);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(30, LCDV_MID);
    M5.Lcd.print(" Cal-1  ");
    gyroYoffset = 0.0;
    for (int i = 0; i < N_CAL1; i++) {
        readGyro();
        gyroYoffset += gyroYdata;
        delay(9);
    }
    gyroYoffset /= (float)N_CAL1;
    M5.Lcd.fillScreen(BLACK);
    digitalWrite(LED, HIGH);
}

void calib2() {
    resetVar();
    resetMotor();
    digitalWrite(LED, LOW);
    calDelay(80);
    M5.Lcd.setCursor(30, LCDV_MID);
    M5.Lcd.println(" Cal-2  ");
    accXoffset  = 0.0;
    gyroZoffset = 0.0;
    for (int i = 0; i < N_CAL2; i++) {
        readGyro();
        accXoffset += accXdata;
        gyroZoffset += gyroZdata;
        delay(9);
    }
    accXoffset /= (float)N_CAL2;
    gyroZoffset /= (float)N_CAL2;
    M5.Lcd.fillScreen(BLACK);
    digitalWrite(LED, HIGH);
}

void calDelay(int n) {
    for (int i = 0; i < n; i++) {
        getGyro();
        delay(9);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  GYRO / ACCELEROMETER READING
// ═══════════════════════════════════════════════════════════════════════════
void readGyro() {
    float gX, gY, gZ, aX, aY, aZ;
    M5.Imu.getGyroData(&gX, &gY, &gZ);
    M5.Imu.getAccelData(&aX, &aY, &aZ);
    gyroYdata = gX;
    gyroZdata = -gY;
    gyroXdata = -gZ;
    accXdata  = aZ;
    accZdata  = aY;
}

void getGyro() {
    readGyro();
    varOmg = (gyroYdata - gyroYoffset);           // unit: deg/sec
    yawAngle += (gyroZdata - gyroZoffset) * clk;  // unit: deg
    varAng += (varOmg + ((accXdata - accXoffset) * 57.3 - varAng) * cutoff) *
              clk;  // complementary filter
}

// ═══════════════════════════════════════════════════════════════════════════
//  PID CONTROL PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════
void resetPara() {
    Kang          = 37.0;
    Komg          = 0.84;
    KIang         = 800.0;
    Kyaw          = 4.0;
    Kdst          = 85.0;
    Kspd          = 2.7;
    mechFactL     = 0.45;
    mechFactR     = 0.45;
    punchPwr      = 20;
    punchDur      = 1;
    fbBalance     = -3;
    motorDeadband = 10;
    maxPwr        = 120;
    punchPwr2     = max(punchPwr, motorDeadband);
}

// ═══════════════════════════════════════════════════════════════════════════
//  DRIVE (MAIN BALANCE LOOP)
// ═══════════════════════════════════════════════════════════════════════════
void drive() {
    if (abs(moveRate) > 0.1)
        spinFact = constrain(-(powerR + powerL) / 10.0, -1.0, 1.0);  // moving
    else
        spinFact = 1.0;  // standing
    if (spinContinuous)
        spinTarget += spinStep * spinFact;
    else {
        if (spinTarget < spinDest) spinTarget += spinStep;
        if (spinTarget > spinDest) spinTarget -= spinStep;
    }
    moveTarget += moveStep * (moveRate + (float)fbBalance / 100.0);
    varSpd += power * clk;
    varDst += Kdst * (varSpd * clk - moveTarget);
    varIang += KIang * varAng * clk;
    power =
        varIang + varDst + (Kspd * varSpd) + (Kang * varAng) + (Komg * varOmg);
    if (abs(power) > 1000.0)
        counterOverPwr += 1;
    else
        counterOverPwr = 0;
    if (counterOverPwr > maxOvp) return;
    power    = constrain(power, -maxPwr, maxPwr);
    yawPower = (yawAngle - spinTarget) * Kyaw;
    powerR   = power - yawPower;
    powerL   = power + yawPower;

    // ── Left motor ──
    ipowerL      = (int16_t)constrain(powerL * mechFactL, -maxPwr, maxPwr);
    int16_t mdbn = -motorDeadband;
    int16_t pp2n = -punchPwr2;
    if (ipowerL > 0) {
        if (motorLdir == 1)
            punchCountL = constrain(++punchCountL, 0, 100);
        else
            punchCountL = 0;
        motorLdir = 1;
        if (punchCountL < punchDur)
            drvMotorL(max(ipowerL, punchPwr2));
        else
            drvMotorL(max(ipowerL, (int16_t)motorDeadband));
    } else if (ipowerL < 0) {
        if (motorLdir == -1)
            punchCountL = constrain(++punchCountL, 0, 100);
        else
            punchCountL = 0;
        motorLdir = -1;
        if (punchCountL < punchDur)
            drvMotorL(min(ipowerL, pp2n));
        else
            drvMotorL(min(ipowerL, mdbn));
    } else {
        drvMotorL(0);
        motorLdir = 0;
    }

    // ── Right motor ──
    ipowerR = (int16_t)constrain(powerR * mechFactR, -maxPwr, maxPwr);
    if (ipowerR > 0) {
        if (motorRdir == 1)
            punchCountR = constrain(++punchCountR, 0, 100);
        else
            punchCountR = 0;
        motorRdir = 1;
        if (punchCountR < punchDur)
            drvMotorR(max(ipowerR, punchPwr2));
        else
            drvMotorR(max(ipowerR, (int16_t)motorDeadband));
    } else if (ipowerR < 0) {
        if (motorRdir == -1)
            punchCountR = constrain(++punchCountR, 0, 100);
        else
            punchCountR = 0;
        motorRdir = -1;
        if (punchCountR < punchDur)
            drvMotorR(min(ipowerR, pp2n));
        else
            drvMotorR(min(ipowerR, mdbn));
    } else {
        drvMotorR(0);
        motorRdir = 0;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  MOTOR CONTROL (I2C to STM32 @ 0x38)
// ═══════════════════════════════════════════════════════════════════════════
void drvMotorL(int16_t pwm) {
    drvMotor(0, (int8_t)constrain(pwm, -127, 127));
}

void drvMotorR(int16_t pwm) {
    drvMotor(1, (int8_t)constrain(-pwm, -127, 127));
}

void drvMotor(byte ch, int8_t sp) {
    Wire.beginTransmission(0x38);
    Wire.write(ch);
    Wire.write(sp);
    Wire.endTransmission();
}

void resetMotor() {
    drvMotorR(0);
    drvMotorL(0);
    counterOverPwr = 0;
}

// ═══════════════════════════════════════════════════════════════════════════
//  STATE MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════
void resetVar() {
    power          = 0.0;
    moveTarget     = 0.0;
    moveRate       = 0.0;
    spinContinuous = false;
    spinDest       = 0.0;
    spinTarget     = 0.0;
    spinStep       = 0.0;
    yawAngle       = 0.0;
    varAng         = 0.0;
    varOmg         = 0.0;
    varDst         = 0.0;
    varSpd         = 0.0;
    varIang        = 0.0;
    ctrlMoveRate   = 0.0;
    ctrlSpinStep   = 0.0;
}

// ═══════════════════════════════════════════════════════════════════════════
//  UI & MODE CONTROL
// ═══════════════════════════════════════════════════════════════════════════
void checkButtonP() {
    byte pbtn = M5.Axp.GetBtnPress();
    if (pbtn == 2)
        calib1();        // short push
    else if (pbtn == 1)
        setMode(true);   // long push
}

void setMode(bool inc) {
    if (inc) demoMode = ++demoMode % 2;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(30, 5);
    if (demoMode == 0)
        M5.Lcd.print("Stand ");
    else if (demoMode == 1)
        M5.Lcd.print("Demo ");

    // Re-show BT status after screen clear
    M5.Lcd.setCursor(5, 25);
    if (controllerConnected) {
        M5.Lcd.setTextColor(GREEN, BLACK);
        M5.Lcd.print("BT OK ");
    } else {
        M5.Lcd.setTextColor(YELLOW, BLACK);
        M5.Lcd.print("BT.. ");
    }
    M5.Lcd.setTextColor(WHITE, BLACK);
}

void startDemo() {
    moveRate       = 1.0;
    spinContinuous = true;
    spinStep       = -40.0 * clk;
}

// ═══════════════════════════════════════════════════════════════════════════
//  DISPLAY & SERIAL
// ═══════════════════════════════════════════════════════════════════════════
void dispBatVolt() {
    M5.Lcd.setCursor(35, LCDV_MID);
    vBatt = M5.Axp.GetBatVoltage();
    M5.Lcd.printf("%4.2fv ", vBatt);
}

void sendStatus() {
    Console.printf("%lu stand=%d accX=%.2f power=%.2f ang=%.2f move=%.2f spin=%.2f tof=%dmm\n",
                   millis() - time0, standing, accXdata, power, varAng, moveRate, spinStep, tofDistance);
}

// ═══════════════════════════════════════════════════════════════════════════
//  TOF OBSTACLE DETECTION
// ═══════════════════════════════════════════════════════════════════════════
void checkObstacle() {
    if (!tofAvailable) return;

    // Non-blocking: check if new measurement data is available
    uint8_t intStatus = tofSensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if ((intStatus & 0x07) == 0) return;  // No new data yet

    // Data is ready — read range and clear interrupt
    tofDistance = tofSensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    tofSensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

    // Check for valid reading (VL53L0X returns 8190 for out-of-range)
    if (tofDistance > 8000) {
        obstacleDetected = false;
        M5.Beep.mute();
        return;
    }

    if (tofDistance < OBSTACLE_THRESHOLD_MM) {
        if (!obstacleDetected) {
            Console.printf("OBSTACLE at %d mm!\n", tofDistance);
        }
        obstacleDetected = true;
        M5.Beep.tone(BEEP_FREQ);
    } else {
        if (obstacleDetected) {
            Console.println("Obstacle cleared");
        }
        obstacleDetected = false;
        M5.Beep.mute();
    }
}
