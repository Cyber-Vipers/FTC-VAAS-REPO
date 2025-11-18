package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Line;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Point;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

@Config
public class Drivetrain {
    private Telemetry telemetry;

    public enum DriveState {
        DRIVE,
        PID,
        PARK,
        CLOSEPID,
        HOLD_POS,
    }
    public DriveState driveState = DriveState.DRIVE;

    //public OTOS otos;
    public Pinpoint pinpoint;

    private double drivetrainWidth = 15;
    private double drivetrainLength = 15;

    private double turretOffsetDist = -6;

    private final DcMotorEx backLeft, frontLeft, backRight, frontRight;
    private DcMotorEx[] motors;




    //Pose Shiz
    private Pose2D pose = new Pose2D(0, 0, 0);
    public double xError, yError, hError;
    private double targetX, targetY, targetH;
    private boolean goThroughPoint = false;
    private boolean drift = false;
    double maxSpeed = 1;


    //Lines
    private Line closeRight = new Line(1, 0, 72);
    private Line closeLeft = new Line(-1, -72, 0);
    private Line farRight = new Line(-1, new Point(0, -48), 0, 24);
    private Line farLeft = new Line(1, new Point(0, -48), -24, 0);

    //Points
    private Point topLeft;
    private Point topRight;
    private Point bottomLeft;
    private Point bottomRight;

    public Drivetrain(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

       // otos = new OTOS(hwMap, telemetry);
        pinpoint = new Pinpoint(hwMap, telemetry);

        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backRight = hwMap.get(DcMotorEx.class, "backRight");
        motors = new DcMotorEx[]{backLeft, frontLeft, backRight, frontRight};

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void update() {
        pose = pinpoint.getPose();
        double xChange = targetX - pose.x;
        double yChange = targetY - pose.y;
        xError = xChange;
        yError = -yChange;

        hError = -Math.toDegrees(angleWrap((targetH - pose.h)));

        switch (driveState) {
            case PID:
                PID();
                if (atPoint() && !goThroughPoint) {
                    driveState = DriveState.CLOSEPID;
                } else if (atPoint()) {
                    driveState = DriveState.HOLD_POS;
                }
                break;
            case DRIVE:
                break;
            case HOLD_POS:
                PID();
            case PARK:
                break;
            case CLOSEPID:
                PID();
                if (atClosePoint()) {
                    driveState = DriveState.HOLD_POS;
                    setDrivePower(0, 0, 0, 0);
                }
        }

//        topRight.updatePoint(pose.x + Math.cos(Math.toRadians(pose.h) * drivetrainWidth/2), pose.y + Math.sin(Math.toRadians(pose.h) * drivetrainLength/2));
//        topLeft.updatePoint(pose.x - Math.cos(Math.toRadians(pose.h) * drivetrainWidth/2), pose.y + Math.sin(Math.toRadians(pose.h) * drivetrainLength/2));
//        bottomRight.updatePoint(pose.x + Math.cos(Math.toRadians(pose.h) * drivetrainWidth/2), pose.y - Math.sin(Math.toRadians(pose.h) * drivetrainLength/2));
//        bottomLeft.updatePoint(pose.x - Math.cos(Math.toRadians(pose.h) * drivetrainWidth/2), pose.y - Math.sin(Math.toRadians(pose.h) * drivetrainLength/2));

        telemetry.addData("X Target: ", targetX);
        telemetry.addData("Y Target: ", targetY);
        telemetry.addData("H Target: ", targetH);

        telemetry.addData("X Pos: ", pose.x);
        telemetry.addData("Y Pos: ", pose.y);
        telemetry.addData("Heading: ", Math.toDegrees(pose.h));

        telemetry.addData("X Error: ", xError);
        telemetry.addData("Y Error: ", yError);
        telemetry.addData("Heading Error: ", hError);

        telemetry.addData("Drive State: ", driveState);
    }


    public void setDrivePower(double lFPower, double rFPower, double lBPower, double rBPower) {
        frontLeft.setPower(lFPower * maxSpeed);
        frontRight.setPower(rFPower * maxSpeed);
        backLeft.setPower(lBPower * maxSpeed);
        backRight.setPower(rBPower * maxSpeed) ;
    }

    public void drive(Gamepad gamepad) {
        double lyInput = -(gamepad.left_stick_y);
        double ryInput = -(gamepad.right_stick_y);
        double lxInput = (gamepad.left_stick_x);
        double rxInput = (gamepad.right_stick_x);

        setDrivePower(
                lyInput + ryInput + rxInput + lxInput,
                lyInput + ryInput - rxInput - lxInput,
                lyInput + ryInput + rxInput - lxInput,
                lyInput + ryInput - rxInput + lxInput
        );
    }

    public static double xP = 0.085;

    public static double yP = 0.085;

    public static double hP = 0.1;

    public PID pidX = new PID(xP, 0, 0);
    public PID pidY = new PID(yP, 0, 0);
    public PID pidH = new PID(hP, 0, 0);

    public void PID() {
        pidH.updatePID(hP, 0, 0);
        pidX.updatePID(xP, 0, 0);
        pidY.updatePID(yP, 0, 0);

        double x = pidX.update(xError);
        double y = pidY.update(yError);
        double h = pidH.update(hError);

        double y_rotated = x * Math.cos(pose.h) - y * Math.sin(pose.h);
        double x_rotated = x * Math.sin(pose.h) + y * Math.cos(pose.h);

        double lFPower = y_rotated + x_rotated + h;
        double rFPower = y_rotated - x_rotated - h;
        double lBPower = y_rotated - x_rotated + h;
        double rBPower = y_rotated + x_rotated - h;

        double maxPower = Math.max(1.0, Math.max(Math.abs(lFPower), Math.max(Math.abs(rFPower), Math.max(Math.abs(lBPower), Math.abs(rBPower)))));
        lFPower /= maxPower;
        rFPower /= maxPower;
        lBPower /= maxPower;
        rBPower /= maxPower;

        telemetry.addData("LF:",lFPower);
        telemetry.addData("LB:",lBPower);

        telemetry.addData("RF:",rFPower);
        telemetry.addData("RB:",rBPower);

        setDrivePower(lFPower, rFPower, lBPower, rBPower);
    }


    public boolean atPoint() {
        return (Math.abs(xError) < 5 && Math.abs(yError) < 5 && Math.abs(hError) < 4);
    }

    public boolean atClosePoint() {
        return (Math.abs(xError) < 1.5 && Math.abs(yError) < 1.5 && Math.abs(hError) < 1.5);
    }

    public boolean isBusy() {
        return (driveState != DriveState.PARK && driveState != DriveState.HOLD_POS);
    }

    public Pose2D poseFromCamera(Pose2D turretPose) {
        double robotHeadingInRadians = Math.toRadians(pose.h);
        double offsetX = Math.cos(robotHeadingInRadians) * turretOffsetDist;
        double offsetY = Math.sin(robotHeadingInRadians) * turretOffsetDist;
        pose = new Pose2D(offsetX, offsetY, pose.h);
        return pose;
    }

    public void goToPoint(Pose2D targetPose, boolean through, double maxSpeed) {
        targetX = targetPose.x;
        targetY = targetPose.y;
        targetH = targetPose.h;
        driveState = DriveState.PID;
        goThroughPoint = through;
        this.maxSpeed = maxSpeed;
    }

    public Point[] closestPoints() {
        if (pose.y > -24) {
            if (true) {
            } else {

            }
        } else {

        }
        return null;
    }

    public boolean robotInZone() {
        return (pointInZone(pose.x + Math.cos(Math.toRadians(pose.h) * drivetrainWidth/2), pose.y + Math.sin(Math.toRadians(pose.h) * drivetrainLength/2))
                || pointInZone(pose.x - Math.cos(Math.toRadians(pose.h) * drivetrainWidth/2), pose.y + Math.sin(Math.toRadians(pose.h) * drivetrainLength/2))
                || pointInZone(pose.x + Math.cos(Math.toRadians(pose.h) * drivetrainWidth/2), pose.y - Math.sin(Math.toRadians(pose.h) * drivetrainLength/2))
                || pointInZone(pose.x - Math.cos(Math.toRadians(pose.h) * drivetrainWidth/2), pose.y - Math.sin(Math.toRadians(pose.h) * drivetrainLength/2)));
    }

    public boolean lineInZone(Point p1, Point p2) {
        Line line = new Line(p1.slope(p2), p1);
        double lowX = Math.min(p1.x, p2.x);
        double highX = Math.max(p1.x, p2.x);
        return line.intersectWithinRange(closeLeft, lowX, highX) || line.intersectWithinRange(closeRight, lowX, highX) || line.intersectWithinRange(farRight, lowX, highX) || line.intersectWithinRange(farLeft, lowX, highX);
    }

    public boolean pointInZone(double x, double y) {
        return (y >= Math.abs(x) || (Math.abs(x) <= 24 && y <= Math.abs(x)-48));
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

}
