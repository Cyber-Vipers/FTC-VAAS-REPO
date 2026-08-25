package org.firstinspires.ftc.teamcode.Idontcare.teleop;

import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="North Teleop (ONLY USE WITH REGION NORTH BOT)", group = "Region North")
public class NorthTeleop extends OpMode {
    public static DcMotor frontLeft,frontRight,backRight,backLeft,intake,lift;
    DcMotorEx flywheel;
    public static CRServo feeder;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class,"intake");
        lift = hardwareMap.get(DcMotor.class,"lift");
        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
        feeder = hardwareMap.get(CRServo.class,"feeder");
        telemetry.addLine("THIS OP MODE WILL NOT RUN OFF THE TABLE WHEN RAN");
        telemetry.addData("Status", "Initialized");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public static double flywheelModifier = 0;
    public static double liftMax = 1390;
    public static double liftMin = 0;
    public static double maxFlywheel = 5500;
    public static double rotmodifier = 0.7;
    @Override
    public void loop() {
        int currentLiftPosition = lift.getCurrentPosition();
        telemetry.addData("Lift Position",currentLiftPosition);
        telemetry.addData("Flywheel Velocity",flywheel.getVelocity());
        telemetry.addData("Flywheel Modifier",flywheelModifier);
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * rotmodifier);
        if (gamepad1.dpadLeftWasPressed()) {
            // increase flywheel speed
            if (flywheelModifier + 100 <= 1500) {
                flywheelModifier = flywheelModifier + 100;

            } else {
                flywheelModifier = 1500;
            }
        }

        if (gamepad1.dpadRightWasPressed()) {
            // decrease flywheel speed
            flywheelModifier -= 100;
        }
        boolean liftup = gamepad1.dpad_up && currentLiftPosition + 100 <= liftMax;

        telemetry.addData("Lift up condition", liftup);
        if (liftup) {
            lift.setTargetPosition(currentLiftPosition + 100);
            lift.setPower(1);
        }
        boolean liftdown = gamepad1.dpad_down && currentLiftPosition - 100 >= liftMin;
        telemetry.addData("Lift down condition", liftdown);

        if (liftdown) {
            lift.setTargetPosition(currentLiftPosition - 100);
            lift.setPower(1);
        }
        if (gamepad1.b) {
            //stop flywheel
            flywheel.setVelocity(0 + flywheelModifier);
        }
        if (gamepad2.start) {
            stop();
        }
//        if (!liftdown && !liftup) {
//            lift.setPower(0);
//        }
        if (gamepad1.y) {
            //spin up flywheel
            flywheel.setVelocity(1200 + flywheelModifier);
        }
        if (gamepad1.a) {
            //Intake On
            intake.setPower(1);
        }
        if (gamepad1.b) {
            //Intake Off
            intake.setPower(0);
        }
        if (gamepad1.right_trigger > 0.5) {
            rotmodifier = 0.4;
        } else {
            rotmodifier = 0.7;
        }
        if (gamepad1.left_trigger > 0.5) {
            feeder.setPower(1);
        } else {
            feeder.setPower(-1);
        }
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double leftFrontPower = (forward + strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double leftBackPower = (forward - strafe + rotate) / denominator;
        double rightBackPower = (forward + strafe - rotate) / denominator;

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);

    }
}
