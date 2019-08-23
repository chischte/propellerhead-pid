void GetPotentiometerValues()
{
  /*
   * *****************************************************************************
   * READ ALL POTENTIOMETERS AND CONVERT THEM INTO DIGITAL VALUES
   * read only one pot per cycle (increase cycle speed)
   * *****************************************************************************
   */
  switch (oneAtATime)
  // one analogRead per cycle is slow enough
  {
  case 1:
    Kp_Factor = analogRead(P_VALUE_POT);
    Kp_Factor = map(Kp_Factor, 0, 1023, 0, Kp_Max);
    break;

  case 2:
    Ki_Factor = analogRead(I_VALUE_POT);
    if (Ki_Factor <= 1)
    {
      rpm_I = 0;  // reset i-value
    }
    Ki_Factor = map(Ki_Factor, 0, 1023, 0, Ki_Max);
    break;

  case 3:
    Kd_Factor = analogRead(D_VALUE_POT);
    Kd_Factor = map(Kd_Factor, 0, 1023, 0, Kd_Max);
    break;

  case 4:
    rpm_Max = analogRead(rpm_Max_POT);
    break;
  }
  oneAtATime++;
  if (oneAtATime == 5)
  {
    oneAtATime = 1;
  }
}
