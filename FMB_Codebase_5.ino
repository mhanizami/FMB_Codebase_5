// Four Motor Bob Prototype Code
// Author: Hamza Asif
// Version: 0.5
// Comments: Code uses Hardware Encoders and Hardware PWM of the Teensy 4.x
// Peripherals: 2x Servo Motors, 2x DC Motors
// Date: 09/30/2024 (Edits)

#define _PWM_LOGLEVEL_ 0 // A parameter for  the Teensy_PWM.h file

// Libraries in use:
#include "Teensy_PWM.h"
#include "QuadEncoder.h"
#include <Servo.h>

//-----------------------------------------------------------------//
#define USING_FLEX_TIMERS true  // PWM Library will be using the Flex Timer Pins only. For more info read comments in Teensy_PWM.h

// PMDC Direction Pins:
#define lMotorDirA 7
#define lMotorDirB 8
#define rMotorDirA 11
#define rMotorDirB 10

// PMDC PWM Pins:
uint32_t PWM_Pins[] = { 6, 4 };

// Servo Motor Control Pins:
#define lServoPin 24
#define rServoPin 25

//------------------------------------------------------------------//
// Creating Quadrature Encoder Objects. Total of 4 hardware quadrature encoders allowed in the teensy 4.1:
QuadEncoder lEncoder(1, 0, 1, 0);  // 1 of 4, on pins 0 and 1 and no pullup
QuadEncoder rEncoder(2, 2, 3, 0);  // 2 of 4, on pins 2 and 3 and no pullup

// Creating Servo Motor Objects
Servo lServo; 
Servo rServo;

//------------------------------------------------------------------//
// Initializing PMDC Direction Pins. Avoid setting xMotorDirAValue = xMotorDirBValue = 1  
volatile bool lMotorDirAValue = 0;
volatile bool lMotorDirBValue = 1;
volatile bool rMotorDirAValue = 1;
volatile bool rMotorDirBValue = 0;

// DC Motors Initialization
float frequency[] = { 2000.0f, 2000.0f }; // Control Pulse Frequency
float dutyCycle[] = { 0.0f, 0.0f };     // Control Pulse Dutycycle

//------------------------------------------------------------------//
// PMDC and Servo Motors' position initialization
volatile long lPosition = 0;
volatile long rPosition = 0;
volatile int16_t lVelocity = 0;
volatile int16_t rVelocity = 0;
volatile float lServoPosition = 0*(M_PI/180); // In radians
volatile float rServoPosition = 0*(M_PI/180); // In radians

//------------------------------------------------------------------//
#define NUM_OF_PINS (sizeof(PWM_Pins) / sizeof(uint32_t)) // typecasting number of pins
Teensy_PWM* PWM_Instance[NUM_OF_PINS]; // Using the PWM_instance of the Teensy_PWM library to set up the desired number of pins

//------------------------------------------------------------------//
// Controller Variables
volatile uint16_t ticksPerRev = 3978; // Encoder Ticks or counts per revolution
volatile uint16_t lKp = 1; // Left Motor's Proportional Gain
volatile uint16_t lKd = 1; // Left Motor's Derivative Gain
volatile uint16_t rKp = 1; // Right Motor's Proportional Gain
volatile uint16_t rKd = 1; // Right Motor's Derivative Gain
volatile int16_t lPosError = 0; // Error in left motor's position
volatile int16_t rPosError = 0; // Error in left motor's position
volatile int16_t lVelError = 0; // Error in left motor's velocity
volatile int16_t rVelError = 0; // Error in left motor's velocity

// Desired Trajectory
volatile float actuationFrequency = 2.0f;
volatile long lDesPosition = 0;
volatile long rDesPosition = 0;
volatile int16_t lDesVelocity = 0;
volatile int16_t rDesVelocity = 0;
volatile float lServoAmp = 0.0f;
volatile float lServoPhase = 0.0f*(M_PI/180);
volatile float lServoBias = 40.0f*(M_PI/180);
volatile float rServoAmp = 0.0f;
volatile float rServoPhase = 0.0f*(M_PI/180);
volatile float rServoBias = 140.0f*(M_PI/180);

// Discrete Sampling parameters
volatile int n = 0;  // discrete time samples for trajectory generation
volatile int N = 100; // control frequency

// PWM Control Variables Predefined here:
volatile int lDeltaPWM = 0; // left PWM calculated as a raw output of the controller
volatile int rDeltaPWM = 0; // right PWM calculated as a raw output of the controller
volatile float lPWM1Hz = 43.0f; // Standardized PWM value corresponding to 1 Hz no load left arm output with power from USB port
volatile float rPWM1Hz = 41.0f; // Standardized PWM value corresponding to 1 Hz no load right arm output with power from USB port
// volatile float lPWM2Hz = 70.0f; // Standardized PWM value corresponding to 2 Hz no load left arm output with power from USB port
// volatile float rPWM2Hz = 70.0f; // Standardized PWM value corresponding to 2 Hz no load right arm output with power from USB port
volatile float lPWM3Hz = 99.0f; // Standardized PWM value corresponding to 3 Hz no load left arm output with power from USB port
volatile float rPWM3Hz = 99.0f; // Standardized PWM value corresponding to 3 Hz no load right arm output with power from USB port
volatile float lPWMAnyHz = actuationFrequency*(lPWM3Hz - lPWM1Hz)/2; // Any PWM value between 3 and 1
volatile float rPWMAnyHz = actuationFrequency*(rPWM3Hz - rPWM1Hz)/2; // Any PWM value between 3 and 1
volatile float lPWM = 0; // Left PWM Value to be sent to the PWM_Teensy Library
volatile float rPWM = 0; // Right PWM Value to be sent to the PWM_Teensy Library

// Servo Motor Control Parameters
volatile float lServoPositionMicros = 0; // In Microseconds
volatile float rServoPositionMicros = 0; // In Microseconds

//////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);// Initalize Serial Comm Baud Rate

  // Encoder Initialation
  lEncoder.setInitConfig(); // Default Encoder Configuration
  lEncoder.init(); // Initialize left Encoder
  rEncoder.setInitConfig(); // Default Encoder Configuration
  rEncoder.init(); // Initialize Right Encoder

  // Attach Servo Motors
  lServo.attach(lServoPin);
  rServo.attach(rServoPin);

  // Start all motors at 0 PWM
  for (uint8_t index = 0; index < NUM_OF_PINS; index++) {
    PWM_Instance[index] = new Teensy_PWM(PWM_Pins[index], frequency[index], 0.0f);

    if (PWM_Instance[index]) {
      PWM_Instance[index]->setPWM();
    }
  }
  
  // Set Default direction for PMDCs
  digitalWrite(lMotorDirA, lMotorDirAValue);
  digitalWrite(lMotorDirB, lMotorDirBValue);
  digitalWrite(rMotorDirA, rMotorDirAValue);
  digitalWrite(rMotorDirB, rMotorDirBValue);
}

void loop() {
  // Desired Position and Velocity for a 1 Hz signal
  lDesPosition = actuationFrequency*(ticksPerRev/N)*n; // Left PMDC Pos
  lDesVelocity = actuationFrequency*(ticksPerRev/N); // Left PMDC Vel
  rDesPosition = actuationFrequency*(ticksPerRev/N)*n; // Right PMDC Pos
  rDesVelocity = actuationFrequency*(ticksPerRev/N); // Right PMDC Vel
  lServoPosition = lServoAmp*sin(2*M_PI*actuationFrequency*((float)n/(float)N) + lServoPhase) + lServoBias; // Left Servo Position in radians
  rServoPosition = rServoAmp*sin(2*M_PI*actuationFrequency*((float)n/(float)N) + rServoPhase) + rServoBias; // Right Servo Position in radians
  // lServoPosition = 0*M_PI/2;
  // rServoPosition = M_PI;

  // Sense the Position and Velocity
  lPosition = lEncoder.read();
  rPosition = rEncoder.read();
  lVelocity = lEncoder.getHoldDifference();
  rVelocity = rEncoder.getHoldDifference();

  // Error Calculation
  lPosError = lDesPosition - lPosition; // Error in left motor's position
  rPosError = rDesPosition - rPosition; // Error in right motor's position
  lVelError = lDesVelocity - lVelocity; // Error in left motor's velocity
  rVelError = rDesVelocity - rVelocity; // Error in right motor's velocity

  // Controller Output
  lDeltaPWM = lKp*lPosError + lKd*lVelError;
  rDeltaPWM = rKp*rPosError + rKd*rVelError;

  // PWM compensation
  lPWM = (float) lDeltaPWM + lPWMAnyHz; // Added the baseline PWM as a basic value so that 
  rPWM = (float) rDeltaPWM + rPWMAnyHz; // we are technically controlling PWM not position

  // PWM saturation
  if (lPWM <= 0.0f){
    lPWM = 0.0f;
  }
  else if (lPWM >= 99.0f){
    lPWM = 99.0f;
  }
  if (rPWM <= 0.0f){
    rPWM = 0.0f;
  }
  else if (rPWM >= 99.0f){
    rPWM = 99.0f;
  }

  // Send updated PWM to PWM library
  dutyCycle[0] = lPWM;
  dutyCycle[1] = rPWM;

  for (uint8_t index = 0; index < NUM_OF_PINS; index++) {
  PWM_Instance[index] = new Teensy_PWM(PWM_Pins[index], frequency[index], dutyCycle[index]);
   if (PWM_Instance[index]) {
      PWM_Instance[index]->setPWM();
   }
  }
  
  lServoPositionMicros = map(lServoPosition, 0.0, M_PI, 1000, 2000);
  rServoPositionMicros = map(rServoPosition, 0.0, M_PI, 1000, 2000);
  lServo.writeMicroseconds(lServoPositionMicros);
  rServo.writeMicroseconds(rServoPositionMicros);

  Serial.print((float)n/(float)N);
  Serial.print("\t");
  Serial.print(lPosition);
  Serial.print("\t");
  Serial.print(lDesPosition);
  Serial.print("\t");
  Serial.print(lServoPosition);
  Serial.print("\t");
  Serial.println(lServoPositionMicros);

  delay(1000/N);

  n = n + 1; // increment the sample interval
}