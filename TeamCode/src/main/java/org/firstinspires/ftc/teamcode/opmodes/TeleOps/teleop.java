package org.firstinspires.ftc.teamcode.opmodes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name = "teleop")
public class teleop extends LinearOpMode {
    private Robot robot;

    private DcMotorEx motor;
    private DcMotorEx motor2;

    private CRServo servo1;
    private CRServo servo2;
    private CRServo servo3;
    private CRServo servo4;

    private Servo axon;

    public double targetVel = 0;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap, telemetry);
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        axon = hardwareMap.get(Servo.class, "axon");

        waitForStart();
        double axonPos = 0.5;
        while (opModeIsActive()) {
//            robot.drivetrain.drive(gamepad1);




            telemetry.addData("Axon Pos: ", gamepad1.right_stick_y);
            telemetry.addData("Motor Power: ", gamepad1.left_stick_y);
            telemetry.addData("Motor Vel: ", motor.getVelocity());
            telemetry.update();
        }
    }

}
