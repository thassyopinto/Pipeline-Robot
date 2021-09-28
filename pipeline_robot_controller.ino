#include <PID_v1.h>
#include <Encoder.h>

// Motor Pins
const int M1 = 22;
const int E1 = 5;
const int M2 = 23;
const int E2 = 6;
const int M3 = 24;
const int E3 = 7;
const int M4 = 25;
const int E4 = 8;

// Encoder Pins
const int EncoderPinA_M1 = 18; // M1 Encoder A (Rolling Motor)
const int EncoderPinB_M1 = 19; // M1 Encoder B (Rolling Motor)
const int EncoderPinA_M2 = 2; // M2 Encoder A (Driving Motor)
const int EncoderPinB_M2 = 3; // M2 Encoder B (Driving Motor)
const int EncoderPinA_M4 = 20; // M4 Encoder A (Rolling Motor)
const int EncoderPinB_M4 = 21; // M4 Encoder B (Rolling Motor)
//const int EncoderPinA_M3 = 0; // M3 Encoder A (Driving Motor)
//const int EncoderPinB_M3 = 0; // M3 Encoder B (Driving Motor)

// Gamepad Parameters
const int GP_BYTE = 14;
const double GP_MID = 127.0;
byte GP_DATA[GP_BYTE];

// Motor Parameters
const int MIN_SPD = 0;
const int MAX_SPD = 100;

// Pulse Width Modulation (PWM) Parameters
const int PWM_CP = 1; // Clock Prescaler (1, 8, 64, 256, 1024) [Timer 4]
const double PWM_F = 31250; // Wave Frequency (Hz)
const double PWM_DC = 0; // Duty Cycle (%)

// Proportional-Integral-Derivative (PID) Controller Parameters
const double PID_KP = 1.0; // PID Kp Gain = 1.0
const double PID_KI = 0.0; // PID Ki Gain = 0.0
const double PID_KD = 0.01; // PID Kd Gain = 0.1
const double PID_MIN = -100; // PID Minimum Output Value
const double PID_MAX = 100; // PID Maximum Output Value
const double PID_ST = 1.0; // PID Sampling Time (ms)
double PID_IN_M1 = 0, PID_IN_M4 = 0; // PID Inputs
double PID_OUT_M1 = 0, PID_OUT_M4 = 0; // PID Outputs
double PID_SP = 0; // PID Setpoint
PID PID_M1(&PID_IN_M1, &PID_OUT_M1, &PID_SP, PID_KP, PID_KI, PID_KD, DIRECT);
PID PID_M4(&PID_IN_M4, &PID_OUT_M4, &PID_SP, PID_KP, PID_KI, PID_KD, DIRECT);

// Encoder Parameters
const int ENC_RES = 3416; // Encoder Resolution
Encoder Encoder_M1(EncoderPinA_M1, EncoderPinB_M1);
//Encoder Encoder_M2(EncoderPinA_M2, EncoderPinB_M2);
//Encoder Encoder_M3(EncoderPinA_M3, EncoderPinB_M3);
Encoder Encoder_M4(EncoderPinA_M4, EncoderPinB_M4);

void setup() {
  // Serial Communication Setup
  Serial.begin(115200);

  // Motor Setup
  pinMode(M1, OUTPUT); // Set Motor Direction Pin
  pinMode(M2, OUTPUT); // Set Motor Direction Pin
  pinMode(M3, OUTPUT); // Set Motor Direction Pin
  pinMode(M4, OUTPUT); // Set Motor Direction Pin
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  digitalWrite(M3, LOW);
  digitalWrite(M4, LOW);

  // Pulse Width Modulation (PWM) Setup
  pinMode(E1, OUTPUT); // Set PWM Output Pin
  pinMode(E2, OUTPUT); // Set PWM Output Pin
  pinMode(E3, OUTPUT); // Set PWM Output Pin
  pinMode(E4, OUTPUT); // Set PWM Output Pin
  TCCR3A = 0; // Reset Timer/Counter Control Register - Channel A
  TCCR3B = 0; // Reset Timer/Counter Control Register - Channel B
  TCCR4A = 0; // Reset Timer/Counter Control Register - Channel A
  TCCR4B = 0; // Reset Timer/Counter Control Register - Channel B
  // ----- Set Clock Prescaler -----
  TCCR3B |= (1 << CS30); // Set Clock Prescaler = 1
  TCCR4B |= (1 << CS40); // Set Clock Prescaler = 1
  // ----- Set Non-Inverting -----
  TCCR3A |= (1 << COM3A1); // Set Non-inverting PWM Mode
  TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1); // Set Non-inverting PWM Mode
  // ----- Set Fast PWM Mode ------
  TCCR3A |= _BV(WGM31); // Fast-PWM (Mode 14, TOP = ICRn) - Part I
  TCCR3B |= _BV(WGM33) | _BV(WGM32); // Fast-PWM (Mode 14, TOP = ICRn) - Part II
  TCCR4A |= _BV(WGM41); // Fast-PWM (Mode 14, TOP = ICRn) - Part I
  TCCR4B |= _BV(WGM43) | _BV(WGM42); // Fast-PWM (Mode 14, TOP = ICRn) - Part II
  ICR3 = F_CPU / (PWM_CP * PWM_F) - 1; // Fast-PWM (Input Capture Register)
  ICR4 = F_CPU / (PWM_CP * PWM_F) - 1; // Fast-PWM (Input Capture Register)
  OCR3A = ICR3 * (PWM_DC / 100.0); // Set Output Compare Register / Pin 5
  OCR4A = ICR4 * (PWM_DC / 100.0); // Set Output Compare Register / Pin 6
  OCR4B = ICR4 * (PWM_DC / 100.0); // Set Output Compare Register / Pin 7
  OCR4C = ICR4 * (PWM_DC / 100.0); // Set Output Compare Register / Pin 8

  // PID Controller Setup
  // --- M1 ---
  PID_M1.SetMode(AUTOMATIC); // Turn On the PID Controller
  PID_M1.SetOutputLimits(PID_MIN, PID_MAX); // Set PID Controller Output Limits
  PID_M1.SetSampleTime(PID_ST); // Set PID Controller Sample Time (ms)
  // --- M4 ---
  PID_M4.SetMode(AUTOMATIC); // Turn On the PID Controller
  PID_M4.SetOutputLimits(PID_MIN, PID_MAX); // Set PID Controller Output Limits
  PID_M4.SetSampleTime(PID_ST); // Set PID Controller Sample Time (ms)
}

void loop() {
  // Update Encoder Readings
  PID_IN_M1 = Encoder_M1.read();
  //PID_IN_M4 = Encoder_M4.read();

  // Update Serial Commands
  if(Serial.available() > 0){
    Serial.readBytes(GP_DATA, GP_BYTE);
    //*// Debug Gamepad
    for (int i = 0; i < GP_BYTE; i++){
      Serial.print(String(GP_DATA[i]));
      Serial.print(" ");
    }
    Serial.println("");
    //*/
    if(GP_DATA[5] == 2 || GP_DATA[5] == 66){
      if(GP_DATA[12] > GP_MID){
        forwardDrive(abs((GP_DATA[12] - GP_MID) / GP_MID));
      }
      else if(GP_DATA[12] < GP_MID){
        reverseDrive(abs((GP_DATA[12] - GP_MID) / GP_MID));
      }
    }
    else{
      fullStop();
    }

    /*if(GP_DATA[5] == 64 || GP_DATA[5] == 66){
      PID_SP = map(GP_DATA[10], 0, 255, 0, ENC_RES) - ENC_RES/2;
    }*/
    if((GP_DATA[5] == 64 || GP_DATA[5] == 66) && GP_DATA[6] == 1){
      PID_SP = PID_SP + ENC_RES/16;
      if(PID_SP > ENC_RES/2){
        PID_SP = ENC_RES/2;
      }
    }
    if((GP_DATA[5] == 64 || GP_DATA[5] == 66) && GP_DATA[6] == 2){
      PID_SP = PID_SP - ENC_RES/16;
      if(PID_SP < -ENC_RES/2){
        PID_SP = -ENC_RES/2;
      }
    }
  }
  // Update PID Outputs
  //PID_SP = map(analogRead(0), 0, 1023, 0, ENC_RES);
  PID_M1.Compute();
  //PID_M4.Compute();
  rollDrive();
}

void rollDrive(){
  if(PID_OUT_M1 > 0){
    digitalWrite(M1, HIGH);
    digitalWrite(M4, LOW);
    motorSpeed(1, PID_OUT_M1);
    motorSpeed(4, PID_OUT_M1);
    //motorSpeed(4, PID_OUT_M4);
  }
  else{
    digitalWrite(M1, LOW);
    digitalWrite(M4, HIGH);
    motorSpeed(1, abs(PID_OUT_M1));
    motorSpeed(4, abs(PID_OUT_M1));
    //motorSpeed(4, abs(PID_OUT_M4));
  }
}

void forwardDrive(double mSpeed){
  digitalWrite(M2, HIGH);
  digitalWrite(M3, LOW);
  motorSpeed(2, mSpeed * MAX_SPD);
  motorSpeed(3, mSpeed * MAX_SPD);
}

void reverseDrive(double mSpeed){
  digitalWrite(M2, LOW);
  digitalWrite(M3, HIGH);
  motorSpeed(2, mSpeed * MAX_SPD);
  motorSpeed(3, mSpeed * MAX_SPD);
}

void fullStop(){
  motorSpeed(1, 0);
  motorSpeed(2, 0);
  motorSpeed(3, 0);
  motorSpeed(4, 0);
}

void motorSpeed(int mNumber, int mSpeed){
  if(mSpeed < MIN_SPD){
    mSpeed = 0;
  }
  switch(mNumber){
    case 1:
      OCR3A = ICR3 * (mSpeed / 100.0); // Set Output Compare Register / Pin 5
      break;
    case 2:
      OCR4A = ICR4 * (mSpeed / 100.0); // Set Output Compare Register / Pin 6
      break;
    case 3:
      OCR4B = ICR4 * (mSpeed / 100.0); // Set Output Compare Register / Pin 7
      break;
    case 4:
      OCR4C = ICR4 * (mSpeed / 100.0); // Set Output Compare Register / Pin 8
      break;
    default:
      break;
  }
}
