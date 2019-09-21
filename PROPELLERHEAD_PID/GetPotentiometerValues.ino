void GetPotentiometerValues() {
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
    KpFactor = analogRead(P_VALUE_POT);
    KpFactor = map(KpFactor, 0, 1023, 0, KpMax);
    break;

  case 2:
    KiFactor = analogRead(I_VALUE_POT);
    if (KiFactor <= 1) {
      rpmI = 0;  // reset i-value
    }
    KiFactor = map(KiFactor, 0, 1023, 0, KiMax);
    break;

  case 3:
    KdFactor = analogRead(D_VALUE_POT);
    KdFactor = map(KdFactor, 0, 1023, 0, KdMax);
    break;

  case 4:
    rpmMax = analogRead(MANUAL_THROTTLE_POT);
    break;
  }
  oneAtATime++;
  if (oneAtATime == 5) {
    oneAtATime = 1;
  }
}
