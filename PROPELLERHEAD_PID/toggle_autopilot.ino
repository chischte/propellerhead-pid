void ToggleAutopilot()
{

  // TOGGLE ON - SWITCH TO PID MODE:
    if (digitalRead(TOGGLE_KNOB) == LOW && autopilot == false)
  {
    pidStartupMode = true; // start PID with gyro angle turned off;
    autopilot = true;
    rpm_P = 0;
    rpm_I = 0; // reset rpm_I sum
    rpm_Sum = 0;
    while (digitalRead(TOGGLE_KNOB) == LOW) // LOW=pushed // keep in loop until button is released
    {
      digitalWrite(ESC_PIN, LOW); // keep motor switched off
    }
  }

  // TOGGLE OFF - SWITCH TO MANUAL MODE:
    if (digitalRead(TOGGLE_KNOB) == LOW && autopilot == true)
  {
    autopilot = false;
    while (digitalRead(TOGGLE_KNOB) == LOW) // LOW=pushed // keep in loop until button is released
    {
      digitalWrite(ESC_PIN, LOW); // keep motor switched off
    }
  }
}
