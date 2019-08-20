void serial_prints() {

    /*V2
     servo.h library replaced with self programme ESC-Motorcontroler

     arduino stopped crashing often after changing the voltage supply from 5V (from the ESC)
     to 12V (from battery) to VIN, maybe the servo.h library would work now too.

     V3
     Code to map the PID potentiometer values exponentially removed, linear is better

     Adding of a new value "min_pwm_autopilot" - in autopilot mode:
     the lowest possible RPM is now a slow rotation. This should make the regulation process a bit smoother
     because the propeller stays always under controll. (To be tested)

     Experiments with activated ESC motor brake to be done, could be usefull for emergency stop
     >>MOTORBRAKE ACTIVATED

     V4
     Implementing a cosinus function to all regulators(P/I/D)

     Implementing pid_startup_mode (slow startup and startup calibration
     remove longs and floats out of pid sectin to be done

     A permanent active D-Regulator seems to work better than the emergency stop algorithm

     //LET GYRO ANGLE DECAY AS SLOW AS POSSIBLE (IT IS NOW CALIBRATED TO 0 AFTER STARTUP)
     //CHECK TO REPLACE ESC "PWM SIGNAL" WITH A "FIXED DELAY SIGNAL"

     V5
     Gyro Angle will be set to 0 if PID Startup is at around 0°, therfore the Gyro Angle decay can stay small

     The ESC does not need a PWM signal at all!!!
     The ESC only reacts to the length of the on pulse.
     The time in between two pulses can be VERY small, a simple digitalWriteLOW digitalWriteHIGH command
     is sufficient as a gap. I tested the time for to digitalwrite operations and it took me
     727356us for 2*65000 digitalWrites that makes about 5.5us per write
     A Gap of 11us is sufficient for the ESC!
     Like this I am able to bring down a program cycle to 1260 to 1570us thats 31-45% faster compared to 2300us

     New Program Organization
     1)set motor pin high and start pwm stopwatch
     2)700us get sensor values
     3)300us pid regulator
     4)14 us toggle autopilot
     5)20us ?calculate motor_pwm
     6)226-536us wait till set motor pin low (stopwatch is between pwm_min and pwm_max (1260 - 1570us)
     7) set motor pin low (time till set off 1260-1570us
     9)restart

     Also like this motor will be set to new speed after maximum 563us after pwm calculation thats 1000us faster(before max 1570)
     like this the program is very close to its maximum performance, limited by the ESC hardware pwm cycle of 1818us.
     Like this the regulating frequency is about 555hz.

     Using an ISR (interupt service routine) to switch the signal for the ESC would make the program even a bit faster, but i prefer
     A structured program cycle with one measurement one calculation and one motorcontrol signal per cycle.

     Further improvement could be achieved by replacing the big propeller with a smaller propeller that allows the motor to react faster to esc changes.
     Further improvement could be achieved by changing to an ESC with "oneshot125"compatibility(switchable in 125us-250us)

     V6
     Angle of the Gyro sensor decays to the anlge value of the accelerometer instead of decaying to zero.
     THIS WAS THE BREAKTHROUGH OF THE WHOLE REGULATION!
     Algorithms for slow startup became obsolete, erased

     V7
     General Clean Up
     removal of 7-digit display parts

     V8
     New Sensor "Adafruit NXP Precision 9DoF Breakout Board"

     V9
     LET THE "PID REGULATOR" LOOP PERMANENTLY RUNNING (for debuging and monitoring), the analog read section
     had therefore to be placed at the end of the program (runtime was > min PWM)

     V10
     In autopilot mode Manual knob adjusts setpoint


     */

    //*****************************************************************************
    //SPEED OPTIMIZATION
    //*****************************************************************************
    //RUNTIME WHOLE PROGRAM:
    //2200 us in Manual mode
    //2300 us in Autopilot mode
    //1600 us (aproximately) and faster since V5
    //motorcontroller:
    //*****************************************************************************
    //  164 us TIME FOR motorcontroller() //with and without serial.flush
    // 1740 us !!! (up to) because of ongoing interupt troubles and arduino crashes I
    //finally decided to program a ESC-control without interrupts. I did not help and
    //I lost over 1500us runtime! ...but also gained a much higher ESC actualisation rate
    //because with the self-programmed code I was able to increase the ESC-PWM frequency
    //and therefore the actualisation speed of the motor from 50Hz (SLOW!) to 550Hz(!)
    //autopilot:
    //*****************************************************************************
    //   14 us TIME FOR toggle autopilot()
    //get_sensor_values:
    //*****************************************************************************
    // 1750 us first measurement
    // 1580 us reduced by 170us by replacing divison with multiplication on several occasions
    // 1420 us reduced by another 160us by removing most of float math
    //  700 us reduced by another 700us! with Wire.setClock(400000); //to speed up I2c from 100 to 400kHz
    //pid_regulator
    //*****************************************************************************
    //12000 us !? first measurement, without lcd print
    // 300 us after bringing values in a reasonable range and reading only one AnalogRead per cycle

    //*****************************************************************************
    //*****************************************************************************
    //SERIAL PRINT SECTION
    //*****************************************************************************
    //*****************************************************************************
    int GYRO_ANGLE_INT = GYRO_ANGLE;

    if (millis() - serialprinttimer > 50) {
        Serial.flush();

        Serial.print(-85);
        Serial.print(",");
        Serial.print(+85);
        Serial.print(",");
        Serial.print(0);
        Serial.print(",");

        //Serial.print(ACC_GRAVITY_RAW);
        //Serial.print(",");
        //Serial.print(ACC_GRAVITY_LPF);
        //Serial.print(",");
        //Serial.print(GRAVITY_ANGLE_LPF);
        //Serial.print(",");

        //Serial.print(GYRO_RAW);
        //Serial.print(",");
        //Serial.print(GYRO_ANGULAR_SPEED);
        //Serial.print(",");
        //Serial.print(GYRO_ANGULAR_SPEED_SMOOTHED);
        //Serial.print(",");
        //Serial.print(GYRO_ANGLE);
        //Serial.print(",");
        Serial.print(GYRO_ANGLE_CALIBRATED);
        //Serial.print(",");
        //Serial.print(GYRO_ANGLE_INT);//INT VALUE FOR FASTER SERIAL PRINT
        //Serial.print(",");

        //Serial.print(",");
        //Serial.println(transmission_delta_t);
        //Serial.print(",");

        //Serial.print(analogRead(d_valuepot));//manual_throttle_pot //p_valuepot//
        //Serial.print(",");
        //Serial.print(digitalRead(toggle_knob));//manual_throttle_pot //p_valuepot//
        //Serial.print(",");
        //Serial.print(",");
        //Serial.print(angle_error);
        //Serial.print(",");
        //Serial.print(cosinus_factor);

        //Serial.print(GYRO_RAW);
        //Serial.print(",");
        //Serial.print(RPM_SUM / 100);
        //Serial.print(",");
        //Serial.print(RPM_P);
        //Serial.print(",");
        //Serial.print(RPM_D/100);
        //Serial.print(",");
        //Serial.print(RPM_D);
        //Serial.print(",");
        //Serial.print(motor_pwm);
        //Serial.print(",");
        //Serial.print(pid_delta_t);
        //Serial.print(",");
        //Serial.println(e);

        serialprinttimer = millis();
        /*
         //ADD A VISUAL "TIMEMARKSPIKE" TO TE SERIALPRINTER
         int timemarkrate = 1000; //AFTER THIS TIME A SPIKE APEARS
         if (millis() > timemarktimer)
         {
         Serial.print(50);
         timemarktimer = millis() + timemarkrate;
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
