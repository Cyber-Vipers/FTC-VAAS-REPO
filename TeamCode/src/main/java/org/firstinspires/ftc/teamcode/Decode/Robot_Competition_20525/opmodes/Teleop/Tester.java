package org.firstinspires.ftc.teamcode.Decode.Robot_Competition_20525.opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Tester extends OpMode {
    // Make one button switch between 3 states up, middle, down
    // int = 1,2,3 double = 1.2,1.3, boolean = true or false
    Servo claw;
    int state = 1;
    @Override
    public void init() {
                claw = hardwareMap.get(Servo.class,"claw");
    }

    @Override
    public void loop() {
     if (gamepad1.b) {
         gamepad1.rumble(100);
         //rumble
         if (state == 1) {
             state = 2;
             claw.setPosition(1);
         } else if (state == 2) {
             state = 3;
             claw.setPosition(0.5);
         } else if (state == 3) {
             state = 1;
             claw.setPosition(0);
         }

     }

    }
}
