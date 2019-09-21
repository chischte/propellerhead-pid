void SerialPrints() {
  //*****************************************************************************
  // SERIAL PRINT SECTION
  //*****************************************************************************
  if (millis() - serialPrintTimer > 50) {
    Serial.flush();

    Serial.print(-85);
    Serial.print(",");
    Serial.print(+85);
    Serial.print(",");
    Serial.print(0);
    Serial.print(",");

    serialPrintTimer = millis();
    /*
     // ADD A VISUAL "SPIKE" TO TE SERIALPRINTER
     int timemarkrate = 1000;
     if (millis() > timeMarkTimer)
     {
     Serial.print(50);
     timeMarkTimer = millis() + timemarkrate;
     }
     else
     {
     Serial.print (0);
     }
     */
    Serial.println(",");
    Serial.flush();
  }
}
