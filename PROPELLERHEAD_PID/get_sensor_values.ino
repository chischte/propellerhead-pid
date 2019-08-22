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
  acc_gravity_raw = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)

      //*****************************************************************************
      //I2C COMMUNICATION - GET RAW SENSOR VALUES FROM FXAS21002C (GYRO)
      //*****************************************************************************
  MPU_addr = 0x21; // I2C address
  Wire.setClock(400000); //to speed up I2c from 100 to 400kHz
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x01); // starting with register 0x03//Library: Adafruit_FXAS21002C.h
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true); // request a total of 14 registers
  gyro_raw = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)

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

  float acc_gravity_max_g_value = 8500; //measured value for full G-force

  //LOW PASS FILTERING / SMOOTHING ACCELERATOR VALUESs
  //*****************************************************************************
  if (acc_gravity_raw == acc_gravity_raw) //FILTER OUT NaN's - ONLY NaN's DON'T PASS THIS TEST!
  {
    if (acc_gravity_raw != 0) //zeros can be caused by sensor errors
    {
      int acc_smoothcounter = 3; //use this value to adapt smoothing
      acc_gravity_lpf = (acc_gravity_lpf * (acc_smoothcounter - 1)
          + acc_gravity_raw) / acc_smoothcounter;
    }
  }
  //CONVERT ACCELERATOR VALUES TO ANGLE // unit [°/10] to avoid float
  //*****************************************************************************
  gravity_angle_lpf_new = asin(acc_gravity_lpf / (acc_gravity_max_g_value))
      * -65; //last factor and addend are experimentally determined

  if (gravity_angle_lpf_new == gravity_angle_lpf_new) //FILTER OUT NaN's - ONLY NaN's DON'T PASS THIS TEST!
  {
    gravity_angle_lpf = gravity_angle_lpf_new;
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
  gyro_angular_speed = gyro_raw * -0.0001 * 90;
  //the last factor is to adjust to the desired the unit //check value via calculated angle

  //SMOOTHING OF GYRO ANGULAR SPEED VALUES FOR THE PID REGULATOR (VIBRATIONS!)
  //*****************************************************************************
  int gyro_smoothcounter = 1; //use this value to adapt smoothing
  gyro_angular_speed_smoothed = (gyro_angular_speed_smoothed
      * (gyro_smoothcounter - 1) + gyro_angular_speed) / gyro_smoothcounter;

  //CALCULATING ANGLE: NEW_ANGLE = OLD_ANGLE + (ANGULAR_SPEED*DELTA_TIME)
  //*****************************************************************************
  transmission_delta_t = micros() - transmission_stopwatch;
  gyro_angle = gyro_angle
      + (gyro_angular_speed * (transmission_delta_t * 0.000001));
  transmission_stopwatch = micros(); //RESTART THE CYCLE STOPWATCH

  //THE GYRO ANGLE SHOULD SLOWLY DECAY TO THE VALUE OF THE ACCELERATOR ANGLE (AUTOCALIBRATION)
  //*****************************************************************************
  int gyro_calibration_smoothvalue = 500; //higher value = slower calibration

  gyro_angle_calibrated = (gyro_angle * (gyro_calibration_smoothvalue - 1)
      + (gravity_angle_lpf)) / gyro_calibration_smoothvalue;
  gyro_angle = gyro_angle_calibrated; //update the angle value for the next loop

  Serial.flush();
}
