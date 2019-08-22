void toggle_autopilot()
{

  //TOGGLE ON - SWITCH TO PID MODE
  //*****************************************************************************
  if (digitalRead(toggle_knob) == LOW && autopilot == false)
  {
    pid_startupmode = true; //start PID with gyro angle turned off;
    autopilot = true;
    RPM_P = 0;
    RPM_I = 0; //reset RPM_I sum
    RPM_SUM = 0;
    while (digitalRead(toggle_knob) == LOW) //LOW=pushed //keep in loop until button is released
    {
      digitalWrite(ESC_pin, LOW); //keep motor switched off
    }
  }

  //TOGGLE OFF - SWITCH TO MANUAL MODE
  //*****************************************************************************
  if (digitalRead(toggle_knob) == LOW && autopilot == true)
  {
    autopilot = false;
    while (digitalRead(toggle_knob) == LOW) //LOW=pushed //keep in loop until button is released
    {
      digitalWrite(ESC_pin, LOW); //keep motor switched off
    }
  }
}
