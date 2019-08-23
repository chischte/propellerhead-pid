void PidRegulator()
{
  //*****************************************************************************
  // CALCULATION OF BASIC VARIABLES FOR THE REGULATOR
  //*****************************************************************************

  angleError = (setpoint - gyroAngleCalibrated); // The setpoint of the regulator is 0° [°]
  newTime = micros();
  pidDeltaT = (newTime - previousTime);
  previousTime = newTime;

  //*****************************************************************************
  // CORE FORMULAS OF THE PID REGULATOR // LIMITATIONS
  //*****************************************************************************

  cosinusFactor = cos((gyroAngleCalibrated) * (2 * 3.14) * 0.00277); // because the motor arm turns in a circle // 0.00277~1/360
  cosinusFactor = Limiter(cosinusFactor, 0.1, 1); // to prevent very small values caused by vibrations

  //-----------------------------------------------------------------------------
  rpmP = KpFactor * angleError * cosinusFactor;
  rpmI = rpmI + angleError * KiFactor * pidDeltaT * cosinusFactor * pow(10, -6);
  rpmD = KdFactor * -0.1 * gyroAngularSpeedSmoothed * cosinusFactor; // reacts directly to the gyroscope
  //-----------------------------------------------------------------------------

  rpmP = Limiter(rpmP, -rpmMax, rpmMax); // (value to be limited, min value, max value)
  rpmI = Limiter(rpmI, -rpmMax, rpmMax);
  rpmD = Limiter(rpmD, -rpmMax, rpmMax);

  //-----------------------------------------------------------------------------
  rpmSum = rpmP + rpmI + rpmD;
  //-----------------------------------------------------------------------------

  rpmSum = Limiter(rpmSum, 0, rpmMax); // rpm_Sum will be requested in the motorpulse calculator section

}

//*****************************************************************************
// FUNCTION TO LIMIT REGULATOR VALUES
//*****************************************************************************
float Limiter(float limited_value, float min_limit, float max_limit)
{
  if (limited_value > max_limit)
  {
    limited_value = max_limit;
  }
  if (limited_value < min_limit)
  {
    limited_value = min_limit;
  }
  return limited_value;
}
