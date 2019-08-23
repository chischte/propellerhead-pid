/*
 * *****************************************************************************
 * PROPELLERHEAD_PID
 * *****************************************************************************
 * Program to control a propeller test rig with a PID regulator
 * *****************************************************************************
 * Michael Wettstein
 * Februar 2018, Zürich
 * *****************************************************************************
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#define ESC_PIN 2

//*****************************************************************************
// MPU_6050 ACCELEROMETER / GYROSCOPE
//*****************************************************************************
// I2C PINS:
// CONNECT SDA TO A4
// CONNECT SCL TO A5
// CONNECT XDA TO D2 //in this case not required
//*****************************************************************************

//*****************************************************************************
// DECLARATION OF VARIABLES / DATA TYPES
//*****************************************************************************
// boolean (true/false)
// byte (0-255)
// int   (-32,768 to 32,767) / unsigned int: 0 to 65,535
// long  (-2,147,483,648 to 2,147,483,647)
// float (6-7 Digits)
//*****************************************************************************

// KNOBS AND POTENTIOMETERS:
const byte TOGGLE_KNOB = 4;
const byte MANUAL_THROTTLE_POT = A0; // potentiometer control motorspeed in manual mode
const byte P_VALUE_POT = A6;
const byte I_VALUE_POT = A1;
const byte D_VALUE_POT = A2;

byte oneAtATime = 1;

boolean autopilot = false;
boolean pidStartupMode = true;

int setpoint = 0; //setpoint of the regulator is 0° //value slightly adapted because of sensor alignment
int speedlimitLow = -200; // set max downwards speed limit [°/s]
int KpMax = 1500; // [RPM /(°/10)] value to scale the regulator potentiometer
int KpFactor;
int KiMax = 8000; // [RPM /(°/10)/s] value to scale the regulator potentiometer
int KiFactor;
int KdMax = 3000; // [RPM /((°/10)/s))] value to scale the regulator potentiometer
int rpmMax = 10000; // just a rough guess
int manualThrottle;
int minPwmManual = 1200; // 1270 motor brake // 1282 motor start
int minPwmAutopilot = 1200;
int maxPwm = 1540; // 1450 sufficient // 1570 limit, faster sounds "unhealthy"
unsigned int motorPwm;

// FOR ESC PROGRAMMING ONLY:
// int min_pwm = 1000; // 1220 Motor off // 1245 motor start
// int maxPwm = ESC_pwm_period;

long rpmP;
long rpmI;
long rpmSum;

unsigned long cycleStopwatch;
unsigned long serialPrintTimer;
unsigned long previousTime;
unsigned long transmissionStopwatch;
unsigned long motorPulseStopwatch;
unsigned long timeMarkTimer;
unsigned long transmissionDeltaT;
unsigned long pidDeltaT;
unsigned long newTime;

float rpmD;
float accGravityRaw;
float gravityAngleLpfNew;
float gyroRaw;
float stopwatch = micros();
float gyroAngle;
float gyroAngleCalibrated;
float cosinusFactor;
float gyroAngularSpeedSmoothed;
float KdFactor;
float accGravityLpf;
float gravityAngleLpf;
float gyroAngularSpeed;
float angleError;

//*****************************************************************************
//******************######**#######*#######*#******#*######********************
//*****************#********#**********#****#******#*#*****#*******************
//******************####****#####******#****#******#*######********************
//***********************#**#**********#****#******#*#*************************
//*****************######***######*****#*****######**#*************************
//*****************************************************************************
void setup()
{
  //***************************************************************************
  // SETUP GYROSCOPE / ACCELEROMETER / MPU_6050
  //***************************************************************************

  // CONNECT SCL TO A5
  // CONNECT SDA TO A4
  // CONNECT XDA TO D2
  Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
  Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

  /* Initialise the sensor */
  if (!gyro.begin())
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    while (1)
      ;
  }

  /* Initialise the sensor */
  if (!accelmag.begin(ACCEL_RANGE_4G))
  {
    while (1)
      ;
  }

  //***************************************************************************
  // SETUP PID REGULATOR
  //***************************************************************************
  pinMode(TOGGLE_KNOB, INPUT_PULLUP);
  pinMode(MANUAL_THROTTLE_POT, INPUT);
  pinMode(P_VALUE_POT, INPUT);
  pinMode(I_VALUE_POT, INPUT);
  pinMode(D_VALUE_POT, INPUT);
  pinMode(ESC_PIN, OUTPUT);
  //***************************************************************************
  Serial.begin(115200);  // start serial connection
  digitalWrite(ESC_PIN, LOW);
  Serial.println("EXIT SETUP");
}

//*****************************************************************************
//********************#*********#####***#####***######*************************
//********************#********#*****#*#*****#**#*****#************************
//********************#********#*****#*#*****#**######*************************
//********************#********#*****#*#*****#**#******************************
//********************#######***#####***#####***#******************************
//*****************************************************************************

void loop()
{

  //***************************************************************************
  // MAIN LOOP
  //***************************************************************************

  motorPulseStopwatch = micros();
  digitalWrite(ESC_PIN, HIGH);
  ToggleAutopilot(); // switch to desired operation mode
  GetSensorValues(); // get the values of the gyroscope / accelerometer
  PidRegulator(); // run the pid regulator loop
  MotorpulseCalculator(); // send desired speed values to the ESC (electronic speed controller)
  while (micros() - motorPulseStopwatch < motorPwm)
  {
    // Wait until ESC pulse has the right length
  }
  digitalWrite(ESC_PIN, LOW);
  GetPotentiometerValues();

  //***************************************************************************
  // ACTIVATE / DEACTIVATE SERIAL PRINTS FOR MONITORING AND DEBUGGING
  //***************************************************************************
  //serial_prints();

  //***************************************************************************
  // STOPWATCH TO READ THE LENGTH OF A PROGRAMCYCLE
  //***************************************************************************
  /*
   long runtime = micros() - stopwatch;
   Serial.println(runtime);
   //delay(300);
   stopwatch = micros();
   */
  //***************************************************************************
}
