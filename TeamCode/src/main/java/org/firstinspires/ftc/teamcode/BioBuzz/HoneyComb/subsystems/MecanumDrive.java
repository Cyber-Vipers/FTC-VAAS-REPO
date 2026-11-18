package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

public class MecanumDrive implements Subsystem {

    public static DcMotorEx frontLeft,frontRight,backLeft,backRight;
    private Gamepad gamepad;

    public static double slowMultiplier = 0.5;
    @Override
    public void update() {
        double forward = gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x * 1.1;
        double rotate = -gamepad.right_stick_x;
        if (gamepad.left_bumper) {
            forward *= slowMultiplier;
            strafe *= slowMultiplier;
            rotate *= slowMultiplier;
        }
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




    public MecanumDrive(HardwareMap hardwareMap,Telemetry telemetry,Gamepad gamepad) {
        frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.gamepad = gamepad;
    }

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry,Gamepad gamepad, String frontRightName, String frontLeftName, String backLeftName, String backRightName) {
        frontRight = hardwareMap.get(DcMotorEx.class,frontRightName);
        frontLeft = hardwareMap.get(DcMotorEx.class, frontLeftName);
        backLeft = hardwareMap.get(DcMotorEx.class, backLeftName);
        backRight = hardwareMap.get(DcMotorEx.class, backRightName);
        this.gamepad = gamepad;
    }
}
