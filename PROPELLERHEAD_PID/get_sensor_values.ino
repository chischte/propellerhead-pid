void get_sensor_values()
{
  //*****************************************************************************
  //I2C COMMUNICATION - GET RAW SENSOR VALUES FROM FXOS8700 (ACC)
  //*****************************************************************************
  int MPU_addr = 0x1F; // I2C address

  Wire.setClock(400000); //to speed up I2c from 100 to 400kHz
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x03);  // starting with register 0x03
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true); // request a total of 14 registers
  ACC_GRAVITY_RAW = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)

      //*****************************************************************************
      //I2C COMMUNICATION - GET RAW SENSOR VALUES FROM FXAS21002C (GYRO)
      //*****************************************************************************
  MPU_addr = 0x21; // I2C address
  Wire.setClock(400000); //to speed up I2c from 100 to 400kHz
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x01); // starting with register 0x03//Library: Adafruit_FXAS21002C.h
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true); // request a total of 14 registers
  GYRO_RAW = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)

      //*****************************************************************************
      //CYCLESTOPWATCH RESULTS FOR THE MPU_6050 SECTION Wire.setClock(100000)
      //1700 micros with and without all float stuff and sin/cos calculations
      //1440 micros for two sensor values out of two I2C reads
      //544micros if only one sensorvalue is communicated via I2C
      //*****************************************************************************

  //*****************************************************************************
  //*****************************************************************************
  //*****************************************************************************
  //ACCELERATOR SENSOR SECTION
  //USE THE ACC_GRAVITY SENSOR AS AN ANGLE SENSOR
  //If the motor arm is horizontal, the ACC_GRAVITY_RAW has 0G
  //If the motor arm is vertical, it has 1G
  //*****************************************************************************
  //*****************************************************************************
  //*****************************************************************************

  float ACC_GRAVITY_MAX_G_VALUE = 8500; //measured value for full G-force

  //LOW PASS FILTERING / SMOOTHING ACCELERATOR VALUESs
  //*****************************************************************************
  if (ACC_GRAVITY_RAW == ACC_GRAVITY_RAW) //FILTER OUT NaN's - ONLY NaN's DON'T PASS THIS TEST!
  {
    if (ACC_GRAVITY_RAW != 0) //zeros can be caused by sensor errors
    {
      int ACC_SMOOTHCOUNTER = 3; //use this value to adapt smoothing
      ACC_GRAVITY_LPF = (ACC_GRAVITY_LPF * (ACC_SMOOTHCOUNTER - 1)
          + ACC_GRAVITY_RAW) / ACC_SMOOTHCOUNTER;
    }
  }
  //CONVERT ACCELERATOR VALUES TO ANGLE // unit [°/10] to avoid float
  //*****************************************************************************
  GRAVITY_ANGLE_LPF_NEW = asin(ACC_GRAVITY_LPF / (ACC_GRAVITY_MAX_G_VALUE))
      * -65; //last factor and addend are experimentally determined

  if (GRAVITY_ANGLE_LPF_NEW == GRAVITY_ANGLE_LPF_NEW) //FILTER OUT NaN's - ONLY NaN's DON'T PASS THIS TEST!
  {
    GRAVITY_ANGLE_LPF = GRAVITY_ANGLE_LPF_NEW;
  }

  //X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X
  //X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X/X

  //*****************************************************************************
  //*****************************************************************************
  //*****************************************************************************
  //GYROSCOPE SENSOR SECTION
  //USE THE GYRO SENSOR VALUES TO CALCULATE AN ANGLE
  //THE OUTPUT OF THE GYRO SENSOR IS ANGULAR SPEED (NOT ACCELERATION)!
  //*****************************************************************************
  //*****************************************************************************
  //*****************************************************************************

  //CONVERT SENSOR UNIT TO ANGULAR SPEED UNIT [°/s]
  //*****************************************************************************
  GYRO_ANGULAR_SPEED = GYRO_RAW * -0.0001 * 90;
  //the last factor is to adjust to the desired the unit //check value via calculated angle

  //SMOOTHING OF GYRO ANGULAR SPEED VALUES FOR THE PID REGULATOR (VIBRATIONS!)
  //*****************************************************************************
  int GYRO_SMOOTHCOUNTER = 1; //use this value to adapt smoothing
  GYRO_ANGULAR_SPEED_SMOOTHED = (GYRO_ANGULAR_SPEED_SMOOTHED
      * (GYRO_SMOOTHCOUNTER - 1) + GYRO_ANGULAR_SPEED) / GYRO_SMOOTHCOUNTER;

  //CALCULATING ANGLE: NEW_ANGLE = OLD_ANGLE + (ANGULAR_SPEED*DELTA_TIME)
  //*****************************************************************************
  transmission_delta_t = micros() - transmission_stopwatch;
  GYRO_ANGLE = GYRO_ANGLE
      + (GYRO_ANGULAR_SPEED * (transmission_delta_t * 0.000001));
  transmission_stopwatch = micros(); //RESTART THE CYCLE STOPWATCH

  //THE GYRO ANGLE SHOULD SLOWLY DECAY TO THE VALUE OF THE ACCELERATOR ANGLE (AUTOCALIBRATION)
  //*****************************************************************************
  int GYRO_CALIBRATION_SMOOTHVALUE = 500; //higher value = slower calibration

  GYRO_ANGLE_CALIBRATED = (GYRO_ANGLE * (GYRO_CALIBRATION_SMOOTHVALUE - 1)
      + (GRAVITY_ANGLE_LPF)) / GYRO_CALIBRATION_SMOOTHVALUE;
  GYRO_ANGLE = GYRO_ANGLE_CALIBRATED; //update the angle value for the next loop

  Serial.flush();
}
