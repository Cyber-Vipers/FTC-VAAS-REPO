package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

public class MecanumDrive implements Subsystem {

    public static DcMotorEx frontLeft,frontRight,backLeft,backRight;
    public Gamepad gamepad;

    @Override
    public void update() {
        double forward = gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double rotate = gamepad.right_stick_x;
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
    }

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry,Gamepad gamepad, String frontRightName, String frontLeftName, String backLeftName, String backRightName) {
        frontRight = hardwareMap.get(DcMotorEx.class,frontRightName);
        frontLeft = hardwareMap.get(DcMotorEx.class, frontLeftName);
        backLeft = hardwareMap.get(DcMotorEx.class, backLeftName);
        backRight = hardwareMap.get(DcMotorEx.class, backRightName);
    }
}
