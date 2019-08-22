void pid_regulator()
{
  //*****************************************************************************
  //CALCULATION OF BASIC VARIABLES FOR THE REGULATOR
  //*****************************************************************************

  angle_error = (setpoint - gyro_angle_calibrated); //The setpoint of the regulator is 0° [°]
  newtime = micros();
  pid_delta_t = (newtime - previoustime);
  previoustime = newtime;

  //*****************************************************************************
  //CORE FORMULAS OF THE PID REGULATOR // LIMITATIONS
  //*****************************************************************************

  cosinus_factor = cos((gyro_angle_calibrated) * (2 * 3.14) * 0.00277); //because the motor arm turns in a circle //0.00277~1/360
  cosinus_factor = limiter(cosinus_factor, 0.1, 1); //to prevent very small values caused by vibrations

  //-----------------------------------------------------------------------------
  rpm_p = kp_factor * angle_error * cosinus_factor;
  rpm_i = rpm_i + angle_error * ki_factor * pid_delta_t * cosinus_factor * pow(10, -6);
  rpm_d = kd_factor * -0.1 * gyro_angular_speed_smoothed * cosinus_factor; //reacts directly to the gyroscope
  //-----------------------------------------------------------------------------

  rpm_p = limiter(rpm_p, -rpm_max, rpm_max); //(value to be limited, min value, max value)
  rpm_i = limiter(rpm_i, -rpm_max, rpm_max);
  rpm_d = limiter(rpm_d, -rpm_max, rpm_max);

  //-----------------------------------------------------------------------------
  rpm_sum = rpm_p + rpm_i + rpm_d;
  //-----------------------------------------------------------------------------

  rpm_sum = limiter(rpm_sum, 0, rpm_max); //rpm_sum will be requested in the motorpulse calculator section

}

//*****************************************************************************
//FUNCTION TO LIMIT REGULATOR VALUES
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
