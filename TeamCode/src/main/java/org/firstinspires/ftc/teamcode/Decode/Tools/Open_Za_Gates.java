package org.firstinspires.ftc.teamcode.Decode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Open Za Gates", group = "Testing Opmodes")
public class Open_Za_Gates extends OpMode {
    public static Servo leftFeeder;
    public static Servo rightFeeder;
    public static ElapsedTime OpenCloseTimer = new ElapsedTime();
    public static double left_feeder_block = 0.25;
    public static double right_feeder_block = 0.5;
    public static double right_feeder_up = 0;
    public static double left_feeder_up = 0.5;
    @Override
    public void init() {
        leftFeeder = hardwareMap.get(Servo.class, "indexLeft");
        rightFeeder = hardwareMap.get(Servo.class, "indexRight");
        OpenCloseTimer.startTime();
    }

    @Override
    public void loop() {
        if (OpenCloseTimer.milliseconds() >= 2000) {
            leftFeeder.setPosition(left_feeder_up);
            rightFeeder.setPosition(right_feeder_up);
            OpenCloseTimer.reset();
            OpenCloseTimer.startTime();
        } else {
            leftFeeder.setPosition(left_feeder_block);
            rightFeeder.setPosition(right_feeder_block);
        }
    }
}
