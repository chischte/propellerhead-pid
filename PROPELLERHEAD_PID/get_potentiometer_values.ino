void get_potentiometer_values()
{
  /*
   * *****************************************************************************
   * READ ALL POTENTIOMETERS AND CONVERT THEM INTO DIGITAL VALUES
   * read only one pot per cycle (increase cycle speed)
   * *****************************************************************************
   */
  switch (one_at_a_time)
  //one analogRead per cycle is slow enough
  {
  case 1:
    kp_factor = analogRead(p_valuepot);
    kp_factor = map(kp_factor, 0, 1023, 0, kp_max);
    break;

  case 2:
    ki_factor = analogRead(i_valuepot);
    if (ki_factor <= 1)
    {
      rpm_i = 0;  //reset i-value
    }
    ki_factor = map(ki_factor, 0, 1023, 0, ki_max);
    break;

  case 3:
    kd_factor = analogRead(d_valuepot);
    kd_factor = map(kd_factor, 0, 1023, 0, kd_max);
    break;

  case 4:
    manual_throttle = analogRead(manual_throttle_pot);
    break;
  }
  one_at_a_time++;
  if (one_at_a_time == 5)
  {
    one_at_a_time = 1;
  }
}
