void MotorpulseCalculator() // ESC = electronic speed controller
{

  //*****************************************************************************
  // MANUAL MODE (controlling motor speed by hand)
  //*****************************************************************************
  if (autopilot == false)
  {
    motorPwm = map(rpmMax, 0, 1023, minPwmManual, maxPwm); // map the potentiometer to the usable range of the motorcontroller
  }

  //*****************************************************************************
  // AUTOPILOT MODE (controlling motor speed with the pid regulator)
  //*****************************************************************************
  if (autopilot == true)
  {
    motorPwm = map(rpmSum, 0, rpmMax, minPwmAutopilot, maxPwm); // map to the usable range of the motorcontroller
    // setpoint = map(rpm_Max, 0, 1023, -70, 70);
    /*
     if (motor_pwm < 1250)
     {
     motor_pwm = 1220; // set propellerspeed off to prevent motor-stutter
     }
     */

  }

  //*****************************************************************************
  // EMERGENCY MEASURES / SPEED LIMITS
  // to prevent fall over of the rig or fast crashing into the end stops
  //*****************************************************************************

  if (gyroAngleCalibrated > 60 && motorPwm > 1380) // limit motor speed if arm points upwards
  {
    motorPwm = 1380;
  }
  // EMERGENCY D-REGULATOR:
  if (gyroAngularSpeed < speedlimitLow) // limit downward speed
  {
    digitalWrite(ESC_PIN, LOW);

    while (gyroAngularSpeed < speedlimitLow / 5)
    {
      motorPulseStopwatch = micros();
      digitalWrite(ESC_PIN, HIGH);
      cosinusFactor = cos((gyroAngleCalibrated) * (2 * 3.14) * 0.00277); // because the motor arm turns in a circle //0.00277~1/360
      cosinusFactor = Limiter(cosinusFactor, 0.1, 1); // to prevent very small values caused by vibrations
      rpmD = 350 * -0.1 * gyroAngularSpeedSmoothed * cosinusFactor; // first factor is to adjust the d-regulator
      rpmD = Limiter(rpmD, 0, rpmMax);
      motorPwm = map(rpmD, 0, rpmMax, minPwmAutopilot, maxPwm);
      GetSensorValues();
      while (micros() - motorPulseStopwatch < motorPwm)
      {
        // Wait until ESC pulse has the right length
      }
      digitalWrite(ESC_PIN, LOW);
    }
  }
}
