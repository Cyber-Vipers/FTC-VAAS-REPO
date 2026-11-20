package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.DriveConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

/**
 * Robot-centric mecanum drive from a gamepad: left stick drives and strafes, right stick X
 * turns, left bumper = slow mode (Constants.DriveConstants.SLOW_MULTIPLIER).
 *
 * While a turn override is set (GoalAim does this to face the goal), it replaces the right
 * stick's turn; forward, strafe and slow mode stay with the driver.
 */
public class MecanumDrive implements Subsystem {

    public final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final Gamepad gamepad;

    private boolean turnOverridden = false;
    private double turnOverride = 0;
    private boolean coast = false;

    @Override
    public void update() {
        if (coast) {
            setPowers(0, 0, 0, 0);
            return;
        }
        double forward = gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x * STRAFE_FACTOR;
        double rotate = -gamepad.right_stick_x;
        if (gamepad.left_bumper) {
            forward *= SLOW_MULTIPLIER;
            strafe *= SLOW_MULTIPLIER;
            rotate *= SLOW_MULTIPLIER;
        }
        if (turnOverridden) rotate = turnOverride;
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        double leftFrontPower = (forward + strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double leftBackPower = (forward - strafe + rotate) / denominator;
        double rightBackPower = (forward + strafe - rotate) / denominator;
        setPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void setPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    /** Use this turn power instead of the right stick until clearTurnOverride(). Same sign as the stick's turn. */
    public void setTurnOverride(double turn) {
        turnOverridden = true;
        turnOverride = turn;
    }

    public void clearTurnOverride() {
        turnOverridden = false;
    }

    public boolean isTurnOverridden() {
        return turnOverridden;
    }

    /** Coast = motors off and free-wheeling, so the robot can be pushed by hand (Shot Tuner hand mode). */
    public void setCoast(boolean coast) {
        if (coast == this.coast) return;
        this.coast = coast;
        DcMotor.ZeroPowerBehavior behavior = coast ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE;
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
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
