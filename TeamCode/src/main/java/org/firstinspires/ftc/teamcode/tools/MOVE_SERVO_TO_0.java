package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "MOVE_SERVO_TO_0", group = "Testing Opmodes")
public class MOVE_SERVO_TO_0 extends OpMode {

    public String servo_name = "modulator";
    public Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, servo_name);
    }

    @Override
    public void loop() {
        servo.setPosition(0);
    }
}
