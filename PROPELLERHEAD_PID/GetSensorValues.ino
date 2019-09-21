void GetSensorValues() {
  //*****************************************************************************
  // I2C COMMUNICATION - GET RAW SENSOR VALUES FROM FXOS8700 (ACC)
  //*****************************************************************************
  int MPU_addr = 0x1F; // I2C address

  Wire.setClock(400000); // to speed up I2c from 100 to 400kHz
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x03);  // starting with register 0x03
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true); // request a total of 14 registers
  accGravityRaw = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)

          //*****************************************************************************
          // I2C COMMUNICATION - GET RAW SENSOR VALUES FROM FXAS21002C (GYRO)
          //*****************************************************************************
  MPU_addr = 0x21; // I2C address
  Wire.setClock(400000); // to speed up I2c from 100 to 400kHz
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x01); // starting with register 0x03// Library: Adafruit_FXAS21002C.h
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true); // request a total of 14 registers
  gyroRaw = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)

          //*****************************************************************************
          // cycleStopwatch RESULTS FOR THE MPU_6050 SECTION Wire.setClock(100000)
          // 1700 micros with and without all float stuff and sin/cos calculations
          // 1440 micros for two sensor values out of two I2C reads
          // 544micros if only one sensorvalue is communicated via I2C
          //*****************************************************************************

  //*****************************************************************************
  // ACCELERATOR SENSOR SECTION
  // USE THE ACC_GRAVITY SENSOR AS AN ANGLE SENSOR
  // If the motor arm is horizontal, the accGravityRaw has 0G
  // If the motor arm is vertical, it has 1G
  //*****************************************************************************

  float acc_gravity_max_g_value = 8500; // measured value for full G-force

  // LOW PASS FILTERING / SMOOTHING ACCELERATOR VALUES:
  if (accGravityRaw == accGravityRaw) // FILTER OUT NaN's - ONLY NaN's DON'T PASS THIS TEST!
          {
    if (accGravityRaw != 0) // zeros can be caused by sensor errors
            {
      int acc_smoothcounter = 3; // use this value to adapt smoothing
      accGravityLpf = (accGravityLpf * (acc_smoothcounter - 1) + accGravityRaw) / acc_smoothcounter;
    }
  }
  // CONVERT ACCELERATOR VALUES TO ANGLE [°]
  gravityAngleLpfNew = asin(accGravityLpf / (acc_gravity_max_g_value)) * -65; // last factor and addend are experimentally determined

  if (gravityAngleLpfNew == gravityAngleLpfNew) // FILTER OUT NaN's - ONLY NaN's DON'T PASS THIS TEST!
          {
    gravityAngleLpf = gravityAngleLpfNew;
  }

  //*****************************************************************************
  // GYROSCOPE SENSOR SECTION
  // USE THE GYRO SENSOR VALUES TO CALCULATE AN ANGLE
  // THE OUTPUT OF THE GYRO SENSOR IS ANGULAR SPEED (NOT ACCELERATION)!
  //*****************************************************************************

  // CONVERT SENSOR UNIT TO ANGULAR SPEED UNIT [°/s]
  gyroAngularSpeed = gyroRaw * -0.0001 * 90;
  // the last factor is to adjust to the desired the unit
  // check value via calculated angle

  // SMOOTHING OF GYRO ANGULAR SPEED VALUES FOR THE PID REGULATOR (VIBRATIONS!)
  int gyro_smoothcounter = 1; // use this value to adapt smoothing
  gyroAngularSpeedSmoothed =
          (gyroAngularSpeedSmoothed * (gyro_smoothcounter - 1) + gyroAngularSpeed)
                  / gyro_smoothcounter;

  // CALCULATING ANGLE: NEW_ANGLE = OLD_ANGLE + (ANGULAR_SPEED*DELTA_TIME)
  transmissionDeltaT = micros() - transmissionStopwatch;
  gyroAngle = gyroAngle + (gyroAngularSpeed * (transmissionDeltaT * 0.000001));
  transmissionStopwatch = micros(); // RESTART THE CYCLE STOPWATCH

  // THE GYRO ANGLE SHOULD SLOWLY DECAY TO THE VALUE OF THE ACCELERATOR ANGLE (AUTOCALIBRATION)
  int gyro_calibration_smoothvalue = 500; // higher value = slower calibration

  gyroAngleCalibrated = (gyroAngle * (gyro_calibration_smoothvalue - 1) + (gravityAngleLpf))
          / gyro_calibration_smoothvalue;
  gyroAngle = gyroAngleCalibrated; // update the angle value for the next loop

  Serial.flush();
}
