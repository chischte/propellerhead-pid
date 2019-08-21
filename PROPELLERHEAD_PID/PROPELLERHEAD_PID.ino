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

//*****************************************************************************
//PRE-SETUP SECTION / PIN LAYOUT
//*****************************************************************************
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#define ESC_pin 2
//*****************************************************************************
//MPU_6050 ACCELEROMETER / GYROSCOPE
//*****************************************************************************
//I2C PINS:
//CONNECT SDA TO A4
//CONNECT SCL TO A5
//CONNECT XDA TO D2 //in this case not required
//*****************************************************************************
//KNOBS AND POTENTIOMETERS
//*****************************************************************************
#define toggle_knob 4
#define manual_throttle_pot A0 // potentiometer control motorspeed in manual mode
#define p_valuepot A6
#define i_valuepot A1
#define d_valuepot A2
//*****************************************************************************
//DECLARATION OF VARIABLES / DATA TYPES
//*****************************************************************************
//boolean (true/false)
//byte (0-255)
//int   (-32,768 to 32,767) / unsigned int: 0 to 65,535
//long  (-2,147,483,648 to 2,147,483,647)
//float (6-7 Digits)
//*****************************************************************************
boolean set_error_stopwatch_clearance = true;
boolean autopilot = false;
boolean pid_startupmode = true;

byte one_at_a_time = 1;

int setpoint = 0; //setpoint of the regulator is 0° //value slightli adapted because of sensor alignment
int speedlimit_low = -200; // SET MAX DOWNWARDS SPEEDLIMIT [°/s]
int startup_counter; //to startup the esc needs a longer delay between the pulses
int kp_max = 1500; //[RPM /(°/10)] USE THIS VALUE TO SCALE THE REGULATOR POTENTIOMETER TO DESIRED RANGE
int kp_factor;
int ki_max = 8000; //[RPM /(°/10)/s] USE THIS VALUE TO SCALE THE REGULATOR POTENTIOMETER TO DESIRED RANGE
int ki_factor;
int kd_max = 3000; //[RPM /((°/10)/s))] UUSE THIS VALUE TO SCALE THE REGULATOR POTENTIOMETER TO DESIRED RANGE
int RPM_MAX = 10000; // just a rough guess
int manual_throttle;
int motor_pwm;
int min_pwm_manual = 1200; // 1270 Motor brake // 1282 Motor start
int min_pwm_autopilot = 1200;
int max_pwm = 1540; //1450 sufficient // 1570 limit, faster sounds "unhealthy"

//FOR ESC PROGRAMMING ONLY:
//int min_pwm = 1000; // 1220 Motor off // 1245 Motor start
//int max_pwm = ESC_pwm_period; //ATTENTION FOR ESC PROGRAMMING ONLY!!!!!

long RPM_P;
long RPM_I;
long RPM_SUM;

unsigned long cyclestopwatch;
unsigned long serialprinttimer;
unsigned long previoustime;
unsigned long stabilizationtime;
unsigned long transmission_stopwatch;
unsigned long motorpulse_stopwatch;
unsigned long timemarktimer;
unsigned long transmission_delta_t;
unsigned long pid_delta_t;
unsigned long newtime;

float ACC_GRAVITY_RAW;
float GRAVITY_ANGLE_LPF_NEW;
float GYRO_RAW;
float stopwatch = micros();
float measured_angle;
float GYRO_ANGLE;
float GYRO_ANGLE_CALIBRATED;
float cosinus_factor;
float GYRO_ANGULAR_SPEED_SMOOTHED;
float RPM_D;
float kd_factor;
float ACC_GRAVITY_LPF;
float GRAVITY_ANGLE_LPF;
float GYRO_ANGULAR_SPEED;
float angle_error;

//*****************************************************************************
//******************######**#######*#######*#******#*######********************
//*****************#********#**********#****#******#*#*****#*******************
//******************####****#####******#****#******#*######********************
//***********************#**#**********#****#******#*#*************************
//*****************######***######*****#*****######**#*************************
//*****************************************************************************
void setup() {
    //***************************************************************************
    //SETUP GYROSCOPE / ACCELEROMETER / MPU_6050
    //***************************************************************************

    //CONNECT SCL TO A5
    //CONNECT SDA TO A4
    //CONNECT XDA TO D2
    Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
    Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

    /* Initialise the sensor */
    if (!gyro.begin()) {
        /* There was a problem detecting the FXAS21002C ... check your connections */
        while (1)
            ;
    }

    /* Initialise the sensor */
    if (!accelmag.begin(ACCEL_RANGE_4G)) {
        while (1)
            ;
    }

    //***************************************************************************
    //SETUP PID REGULATOR
    //***************************************************************************
    pinMode(toggle_knob, INPUT_PULLUP);
    pinMode(manual_throttle_pot, INPUT);
    pinMode(p_valuepot, INPUT);
    pinMode(i_valuepot, INPUT);
    pinMode(d_valuepot, INPUT);
    pinMode(ESC_pin, OUTPUT);
    //***************************************************************************
    Serial.begin(115200);  //start serial connection
    digitalWrite(ESC_pin, LOW);
    Serial.println("EXIT SETUP");
}
//*****************************************************************************
//*****************************************************************************
//********************#*********#####***#####***######*************************
//********************#********#*****#*#*****#**#*****#************************
//********************#********#*****#*#*****#**######*************************
//********************#********#*****#*#*****#**#******************************
//********************#######***#####***#####***#******************************
//*****************************************************************************
//*****************************************************************************

void loop() {

    //***************************************************************************
    //MAIN LOOP
    //***************************************************************************

    motorpulse_stopwatch = micros();
    digitalWrite(ESC_pin, HIGH);
    toggle_autopilot(); //SWITCH TO DESIRED OPERATION MODE
    get_sensor_values(); //GET THE VALUES OF THE GYROSCOPE / ACCELEROMETER
    pid_regulator(); //RUN THE PID REGULATOR LOOP
    motorpulse_calculator(); //SEND DESIRED SPEED VALUES TO THE ESC (ELECTRONIC SPEED CONTROLLER)
    while (micros() - motorpulse_stopwatch < motor_pwm) {
        //Wait until ESC pulse has the right length
    }
    digitalWrite(ESC_pin, LOW);
    get_potentiometer_values();

    //***************************************************************************
    // serial_prints(); //ACTIVATE / DEACTIVATE SERIAL PRINTS FOR MONITORING AND DEBUGGING
    //***************************************************************************
    //STOPWATCH TO READ THE LENGTH OF A PROGRAMCYCLE
    //***************************************************************************
    /*
     long runtime = micros() - stopwatch;
     Serial.println(runtime);
     //delay(300);
     stopwatch = micros();
     */
    //***************************************************************************
}
