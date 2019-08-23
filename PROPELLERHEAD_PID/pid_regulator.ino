void PID_Regulator()
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
  cosinusFactor = limiter(cosinusFactor, 0.1, 1); // to prevent very small values caused by vibrations

  //-----------------------------------------------------------------------------
  rpm_P = Kp_Factor * angleError * cosinusFactor;
  rpm_I = rpm_I + angleError * Ki_Factor * pidDeltaT * cosinusFactor * pow(10, -6);
  rpm_D = Kd_Factor * -0.1 * gyroAngularSpeedSmoothed * cosinusFactor; // reacts directly to the gyroscope
  //-----------------------------------------------------------------------------

  rpm_P = limiter(rpm_P, -rpm_Max, rpm_Max); // (value to be limited, min value, max value)
  rpm_I = limiter(rpm_I, -rpm_Max, rpm_Max);
  rpm_D = limiter(rpm_D, -rpm_Max, rpm_Max);

  //-----------------------------------------------------------------------------
  rpm_Sum = rpm_P + rpm_I + rpm_D;
  //-----------------------------------------------------------------------------

  rpm_Sum = limiter(rpm_Sum, 0, rpm_Max); // rpm_Sum will be requested in the motorpulse calculator section

}

//*****************************************************************************
// FUNCTION TO LIMIT REGULATOR VALUES
//*****************************************************************************
float limiter(float limited_value, float min_limit, float max_limit)
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
