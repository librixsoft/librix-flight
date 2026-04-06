/*
* LibrixFlight (c) 2024 LibrixSoft
* ---------------------------------
* Flight Controller Software
* ---------------------------------
*
* Quadcopter (X Configuration)
* ----------------------------
*
*             - Y Axis -
*
*                RX
*            TL      TR
*            |\     /|
*            ( \   / )
*             \ (_) / 
*              ) _ (   - X Axis -
*             / ( ) \ 
*            ( /   \ )
*            |/     \|
*            BL     BR
*
* AUTHOR: hola@librixsoft.com
* LICENSE: BSD
**/

#include <PinChangeInt.h>
#include <Servo.h>
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Constants
#define THROTTLE_IN_PIN 3 
#define PITCH_IN_PIN 4 
#define ROLL_IN_PIN 5 

#define MOTORTL_OUT_PIN 8
#define MOTORTR_OUT_PIN 9
#define MOTORBR_OUT_PIN 11
#define MOTORBL_OUT_PIN 13

#define OUTPUT_LIMITS 50    // Max PID correction
#define ARM_THRESHOLD 1100  // PWM for stick arming logic

// Variables
uint16_t unThrottleIn, unPitchIn, unRollIn;
int outputTL, outputTR, outputBR, outputBL;
float mpuYaw, mpuPitch, mpuRoll;

bool isArmed = false;
uint32_t armTimer = 0;
int powerTimeout = 0;
int motorGain = 40;

// PID Tuning
double PitchaggKp=0.15, PitchaggKi=0.01, PitchaggKd=0.05;
double PitchconsKp=0.20, PitchconsKi=0.01, PitchconsKd=0.02;

double RollaggKp=0.15, RollaggKi=0.01, RollaggKd=0.05;
double RollconsKp=0.20, RollconsKi=0.01, RollconsKd=0.02;

double pitchSetpoint, pitchInput, pitchOutput;
double rollSetpoint, rollInput, rollOutput;

// Objects
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, PitchconsKp, PitchconsKi, PitchconsKd, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, RollconsKp, RollconsKi, RollconsKd, DIRECT);
MPU6050 mpu;
Servo servoTL, servoTR, servoBR, servoBL;

// ISR Shared Variables
volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared, unPitchInShared, unRollInShared;
uint32_t ulThrottleStart, ulPitchStart, ulRollStart;

// MPU Control
bool dmpReady = false;
uint8_t mpuIntStatus, devStatus, fifoBuffer[64];
uint16_t packetSize, fifoCount;
Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

void setup() {
  Serial.begin(115200);
  
  // PID Init
  pitchInput = rollInput = 0;
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-OUTPUT_LIMITS, OUTPUT_LIMITS);
  rollPID.SetOutputLimits(-OUTPUT_LIMITS, OUTPUT_LIMITS);

  // I2C Init
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24;
  #endif

  // Servos
  servoTL.attach(MOTORTL_OUT_PIN);
  servoTR.attach(MOTORTR_OUT_PIN);
  servoBL.attach(MOTORBL_OUT_PIN);
  servoBR.attach(MOTORBR_OUT_PIN);
  stopMotors();

  // Radio Interrupts
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE); 
  PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch, CHANGE);
  PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll, CHANGE);

  // MPU Init
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220); mpu.setYGyroOffset(76); mpu.setZGyroOffset(-85); mpu.setZAccelOffset(1788);
  
  if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      attachInterrupt(0, dmpDataReady, RISING);
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
  Serial.println("LibrixFlight Ready. Corner stick to ARM.");
}

void loop() {
  if (!dmpReady) return;
  
  // 1. MPU Data Processing
  while (!mpuInterrupt && fifoCount < packetSize);
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) mpu.resetFIFO();
  else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpuRoll = ypr[1] * 180/M_PI;
      mpuPitch = ypr[2] * 180/M_PI;
  }

  // 2. Radio Signal Management
  uint8_t currentFlags = 0;
  if(bUpdateFlagsShared) {
    noInterrupts();
    currentFlags = bUpdateFlagsShared;
    unThrottleIn = unThrottleInShared;
    unPitchIn = unPitchInShared;
    unRollIn = unRollInShared;
    bUpdateFlagsShared = 0;
    interrupts();
    powerTimeout = 0;
  } else {
    powerTimeout++;
  }

  // Failsafe
  if(powerTimeout > 50) {
    isArmed = false;
    stopMotors();
    return;
  }

  // 3. Arming Logic (Corner Sticks: Throttle Low, Pitch/Roll Low Left)
  if (!isArmed) {
      if (unThrottleIn < 1100 && unPitchIn < 1100 && unRollIn < 1100) {
          if (armTimer == 0) armTimer = millis();
          if (millis() - armTimer > 2000) {
              isArmed = true;
              Serial.println("ARMED!");
          }
      } else {
          armTimer = 0;
      }
      stopMotors();
  }

  // 4. PID Computation
  if (isArmed) {
      pitchInput = mpuPitch;
      pitchSetpoint = map(unPitchIn, 1000, 2000, -30, 30);
      double pGap = abs(pitchSetpoint - pitchInput);
      pitchPID.SetTunings(pGap < 5 ? PitchconsKp : PitchaggKp, pGap < 5 ? PitchconsKi : PitchaggKi, pGap < 5 ? PitchconsKd : PitchaggKd);
      pitchPID.Compute();

      rollInput = mpuRoll;
      rollSetpoint = map(unRollIn, 1000, 2000, -30, 30);
      double rGap = abs(rollSetpoint - rollInput);
      rollPID.SetTunings(rGap < 5 ? RollconsKp : RollaggKp, rGap < 5 ? RollconsKi : RollaggKi, rGap < 5 ? RollconsKd : RollaggKd);
      rollPID.Compute();

      // 5. Motor Mixing
      if (unThrottleIn > 1050) {
          outputTL = unThrottleIn + pitchOutput + rollOutput + motorGain;
          outputTR = unThrottleIn + pitchOutput - rollOutput + motorGain;
          outputBL = unThrottleIn - pitchOutput + rollOutput + motorGain;
          outputBR = unThrottleIn - pitchOutput - rollOutput + motorGain;
          
          // Safety Clamping
          outputTL = constrain(outputTL, 1000, 2000);
          outputTR = constrain(outputTR, 1000, 2000);
          outputBL = constrain(outputBL, 1000, 2000);
          outputBR = constrain(outputBR, 1000, 2000);
          
          servoTL.writeMicroseconds(outputTL);
          servoTR.writeMicroseconds(outputTR);
          servoBL.writeMicroseconds(outputBL);
          servoBR.writeMicroseconds(outputBR);
      } else {
          arm(); // Keep idle
      }
  }
}

// ISR Handlers
void calcThrottle() { if(digitalRead(THROTTLE_IN_PIN) == HIGH) ulThrottleStart = micros(); else { unThrottleInShared = (uint16_t)(micros() - ulThrottleStart); bUpdateFlagsShared |= 1; } }
void calcPitch() { if(digitalRead(PITCH_IN_PIN) == HIGH) ulPitchStart = micros(); else { unPitchInShared = (uint16_t)(micros() - ulPitchStart); bUpdateFlagsShared |= 2; } }
void calcRoll() { if(digitalRead(ROLL_IN_PIN) == HIGH) ulRollStart = micros(); else { unRollInShared = (uint16_t)(micros() - ulRollStart); bUpdateFlagsShared |= 4; } }

void stopMotors() {
  servoTL.writeMicroseconds(1000); servoTR.writeMicroseconds(1000);
  servoBL.writeMicroseconds(1000); servoBR.writeMicroseconds(1000);
}

void arm() { stopMotors(); }
