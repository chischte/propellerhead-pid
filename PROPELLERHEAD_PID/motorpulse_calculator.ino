void MotorpulseCalculator() // ESC = ELECTRONIC SPEED CONTROLLER
{

  //*****************************************************************************
  // MANUAL MODE (CONTROLLING MOTOR SPEED BY HAND)
  //*****************************************************************************
  if (autopilot == false)
  {
    motor_pwm = map(rpm_Max, 0, 1023, minPwmManual, maxPwm); // map the potentiometer to the usable range of the motorcontroller
  }

  //*****************************************************************************
  // AUTOPILOT MODE (CONTROLLING MOTOR SPEED WITH THE PID REGULATOR)
  //*****************************************************************************
  if (autopilot == true)
  {
    motor_pwm = map(rpm_Sum, 0, rpm_Max, minPwmAutopilot, maxPwm); // map to the usable range of the motorcontroller
    // setpoint = map(rpm_Max, 0, 1023, -70, 70);
    /*
     if (motor_pwm < 1250)
     {
     motor_pwm = 1220; // set propellerspeed off to prevent motor-stutter
     }
     */

  }

  //*****************************************************************************
  // EMERGENCY MEASURES /SPEED LIMITS
  // to prevent fall over of the rig or fast crashing into the end stops
  //*****************************************************************************

  if (gyroAngleCalibrated > 60 && motor_pwm > 1380) // limit motor speed if arm points upwards
  {
    motor_pwm = 1380;
  }
  // EMERGENCY D-REGULATOR:
   if (gyroAngularSpeed < speedlimitLow) // limit downward speed
  {
    digitalWrite(ESC_PIN, LOW);
    // motor_pwm = maxPwm;
    // EMERGENCY D-REGULATOR:
    while (gyroAngularSpeed < speedlimitLow / 5)
    //while (1 == 1)
    {
      motorPulseStopwatch = micros();
      digitalWrite(ESC_PIN, HIGH);
      cosinusFactor = cos((gyroAngleCalibrated) * (2 * 3.14) * 0.00277); // because the motor arm turns in a circle //0.00277~1/360
      cosinusFactor = limiter(cosinusFactor, 0.1, 1); // to prevent very small values caused by vibrations
      rpm_D = 350 * -0.1 * gyroAngularSpeedSmoothed * cosinusFactor; // first factor is to adjust the d-regulator
      rpm_D = limiter(rpm_D, 0, rpm_Max);
      motor_pwm = map(rpm_D, 0, rpm_Max, minPwmAutopilot, maxPwm);
      GetSensorValues();
      while (micros() - motorPulseStopwatch < motor_pwm)
      {
        // Wait until ESC pulse has the right length
      }
      digitalWrite(ESC_PIN, LOW);
    }
  }
}
