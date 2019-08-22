void motorpulse_calculator() //ESC = ELECTRONIC SPEED CONTROLLER
{

  //*****************************************************************************
  //MANUAL MODE (CONTROLLING MOTOR SPEED BY HAND)
  //*****************************************************************************
  if (autopilot == false)
  {
    motor_pwm = map(manual_throttle, 0, 1023, min_pwm_manual, max_pwm); // map the potentiometer to the usable range of the motorcontroller
  }

  //*****************************************************************************
  //AUTOPILOT MODE (CONTROLLING MOTOR SPEED WITH THE PID REGULATOR)
  //*****************************************************************************
  if (autopilot == true)
  {
    motor_pwm = map(rpm_sum, 0, rpm_max, min_pwm_autopilot, max_pwm); // map to the usable range of the motorcontroller
    //setpoint = map(manual_throttle, 0, 1023, -70, 70);
    /*
     if (motor_pwm < 1250)
     {
     motor_pwm = 1220; //set propellerspeed off to prevent motor-stutter
     }
     */

  }

  //*****************************************************************************
  //EMERGENCY MEASURES /SPEED LIMITS
  //to prevent fall over of the rig or fast crashing into the end stops
  //*****************************************************************************

  if (gyro_angle_calibrated > 60 && motor_pwm > 1380) //limit motor speed if arm points upwards
  {
    motor_pwm = 1380;
  }
  //EMERGENCY D-REGULATOR:
  //*****************************************************************************
  if (gyro_angular_speed < speedlimit_low) //limit downward speed
  {
    digitalWrite(ESC_pin, LOW);
    // motor_pwm = max_pwm;
    //EMERGENCY D-REGULATOR:
    while (gyro_angular_speed < speedlimit_low / 5)
    //while (1 == 1)
    {
      motorpulse_stopwatch = micros();
      digitalWrite(ESC_pin, HIGH);
      cosinus_factor = cos((gyro_angle_calibrated) * (2 * 3.14) * 0.00277); //because the motor arm turns in a circle //0.00277~1/360
      cosinus_factor = limiter(cosinus_factor, 0.1, 1); //to prevent very small values caused by vibrations
      rpm_d = 350 * -0.1 * gyro_angular_speed_smoothed * cosinus_factor; //first factor is to adjust the d-regulator
      rpm_d = limiter(rpm_d, 0, rpm_max);
      motor_pwm = map(rpm_d, 0, rpm_max, min_pwm_autopilot, max_pwm);
      get_sensor_values();
      while (micros() - motorpulse_stopwatch < motor_pwm)
      {
        //Wait until ESC pulse has the right length
      }
      digitalWrite(ESC_pin, LOW);
    }
  }
}
