int pid_regulator()
{
  //*****************************************************************************
  //CALCULATION OF BASIC VARIABLES FOR THE REGULATOR
  //*****************************************************************************

  angle_error = (setpoint - GYRO_ANGLE_CALIBRATED); //The setpoint of the regulator is 0° [°]
  newtime = micros();
  pid_delta_t = (newtime - previoustime);
  previoustime = newtime;

  //*****************************************************************************
  //CORE FORMULAS OF THE PID REGULATOR // LIMITATIONS
  //*****************************************************************************

  cosinus_factor = cos((GYRO_ANGLE_CALIBRATED) * (2 * 3.14) * 0.00277); //because the motor arm turns in a circle //0.00277~1/360
  cosinus_factor = limiter(cosinus_factor, 0.1, 1); //to prevent very small values caused by vibrations

  RPM_P = kp_factor * angle_error * cosinus_factor;

  RPM_I = RPM_I + angle_error * ki_factor * pid_delta_t * cosinus_factor * pow(10, -6);

  RPM_D = kd_factor * -0.1 * GYRO_ANGULAR_SPEED_SMOOTHED * cosinus_factor; //reacts directly to the gyroscope

  RPM_P = limiter(RPM_P, -RPM_MAX, RPM_MAX); //1)VALUE TO BE LIMITED 2)MIN_VALUE 3)MAX_VALUE
  RPM_I = limiter(RPM_I, -RPM_MAX, RPM_MAX);
  RPM_D = limiter(RPM_D, -RPM_MAX, RPM_MAX);

  //RPM_D_SMOOTHED

  RPM_SUM = RPM_P + RPM_I + RPM_D;

  RPM_SUM = limiter(RPM_SUM, 0, RPM_MAX); //RPM_SUM WILL BE REQUESTED IN THE MOTORPULSE CALCULATOR SECTION

}

//*****************************************************************************
//FUNCTION TO LIMIT REGULATOR VALUES
//1)VALUE TO BE LIMITED 2)MIN_VALUE 3)MAX_VALUE
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
