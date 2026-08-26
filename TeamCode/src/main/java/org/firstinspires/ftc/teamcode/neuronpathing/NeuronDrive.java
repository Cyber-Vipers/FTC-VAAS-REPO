package org.firstinspires.ftc.teamcode.neuronpathing;

import static org.firstinspires.ftc.teamcode.neuronpathing.MathFunc.angleWrap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


/**
 * Neuron Pathing's mecanum drivetrain + path follower.
 *
 * Call update() every loop. NeuronDrive will drive a spline (pure pursuit with
 * centripetal correction) if one is active via followSpline()/splineToPoint(),
 * otherwise it holds/approaches the last commanded point with a PID
 * (goToPoint()). Pose comes from a NeuronLocalizer you construct and configure
 * for your own robot's odometry pod placement.
 *
 * This class intentionally only knows about driving and following paths -- no
 * game-specific state (scoring zones, alliance targets, other subsystems).
 * Keep that in your own robot/subsystem classes and read drive.pose from here.
 */
public class NeuronDrive {
    public enum DriveState {
        DRIVE,      // plain teleop / manual control, follower is idle
        PID,        // approaching a single target point
        HOLD_POS,   // arrived, holding position
        CLOSEPID,   // fine correction after arriving
        PARK,       // follower fully disabled
        SPLINE      // actively pursuing a Spline
    }

    public DriveState driveState = DriveState.DRIVE;

    public final NeuronLocalizer localizer;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public Pose2D pose = new Pose2D(0, 0, 0);

    public double strafeError, forwardError, hError;
    private Pose2D targetPoint = new Pose2D(0, 0, 0);
    private boolean goThroughPoint = false;
    private double maxSpeed = 1;

    private final double trackWidth;

    // ---- tunable follower constants (see docs/tuning.md) ----------------------
    /** Pure-pursuit lookahead distance along the spline, in inches. Smaller cuts corners
     *  tighter but can oscillate; larger drives smoother but wider. */
    public static double lookaheadIn = 14;
    /** Distance from the spline's final point at which NeuronDrive stops pure-pursuit
     *  and hands off to the point-holding PID, in inches. */
    public static double splineEndRadiusIn = 14;
    /** Centripetal correction strength while cornering on a spline. Raise if the robot
     *  cuts inside curves; lower if it overshoots/oscillates on curves. */
    public static double centripetalGain = 0.3;
    /** How close (inches / degrees) counts as "arrived" for goToPoint(). */
    public static double pointToleranceIn = 5, headingToleranceDeg = 5;
    /** Tighter tolerance used for the final settle after atPoint() (CLOSEPID). */
    public static double closePointToleranceIn = 2, closeHeadingToleranceDeg = 2;

    public Spline spline = null;
    private int splineIndex = 0;
    private boolean reversedSpline = false;

    public static double strafeP = 0.12, strafeI = 0, strafeD = 0.013;
    public static double forwardP = 0.1, forwardI = 0, forwardD = 0.015;
    public static double hP = 0.05, hI = 0, hD = 0.003;

    public final PID pidForward = new PID(forwardP, forwardI, forwardD);
    public final PID pidStrafe = new PID(strafeP, strafeI, strafeD);
    public final PID pidH = new PID(hP, hI, hD);

    /**
     * @param trackWidthIn distance between left and right wheel contact points, in inches.
     *                     Used for the spline pursuit turn-rate calculation.
     */
    public NeuronDrive(HardwareMap hwMap, NeuronLocalizer localizer, double trackWidthIn,
                        String frontLeftName, String frontRightName,
                        String backLeftName, String backRightName) {
        this.localizer = localizer;
        this.trackWidth = trackWidthIn;

        frontLeft = hwMap.get(DcMotorEx.class, frontLeftName);
        frontRight = hwMap.get(DcMotorEx.class, frontRightName);
        backLeft = hwMap.get(DcMotorEx.class, backLeftName);
        backRight = hwMap.get(DcMotorEx.class, backRightName);

        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        // Standard goBILDA/REV mecanum wiring: left side motors reversed.
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }

    /** Convenience constructor with the common "frontLeft/frontRight/backLeft/backRight" hardware config names. */
    public NeuronDrive(HardwareMap hwMap, NeuronLocalizer localizer, double trackWidthIn) {
        this(hwMap, localizer, trackWidthIn, "frontLeft", "frontRight", "backLeft", "backRight");
    }

    /** Call once per loop. Drives the active spline or holds the last target point. */
    public void update() {
        pose = localizer.getPose();

        if (spline != null) {
            splineIndex = Math.min(spline.points.size() - 1, splineIndex);

            while (splineIndex < spline.points.size() - 1
                    && spline.points.get(splineIndex).distanceToPoint(pose) < lookaheadIn) {
                splineIndex++;
            }
            targetPoint = spline.points.get(Math.min(spline.points.size() - 1, splineIndex));

            if (spline.points.get(spline.points.size() - 1).distanceToPoint(pose) < splineEndRadiusIn) {
                driveState = DriveState.PID;
                targetPoint = spline.points.get(spline.points.size() - 1);
                spline = null;
                splineIndex = 0;
                reversedSpline = false;
            } else {
                driveState = DriveState.SPLINE;
                targetPoint.h = spline.points.get(Math.min(spline.points.size() - 1, splineIndex)).h;
            }
        }

        if (targetPoint == null) return;

        double xError = targetPoint.x - pose.x;
        double yError = targetPoint.y - pose.y;
        double cosH = Math.cos(pose.h);
        double sinH = Math.sin(pose.h);

        strafeError = -xError * sinH + yError * cosH;
        forwardError = xError * cosH + yError * sinH;
        hError = Math.toDegrees(angleWrap(targetPoint.h - pose.h));

        switch (driveState) {
            case SPLINE:
                driveSpline();
                break;
            case PID:
                pidToPoint();
                if (atPoint() && !goThroughPoint) {
                    driveState = DriveState.CLOSEPID;
                } else if (atPoint()) {
                    driveState = DriveState.HOLD_POS;
                }
                break;
            case HOLD_POS:
                pidToPoint();
                break;
            case CLOSEPID:
                pidToPoint();
                if (atClosePoint()) {
                    driveState = DriveState.HOLD_POS;
                }
                break;
            case DRIVE:
            case PARK:
                break;
        }
    }

    private void driveSpline() {
        // Curvature of the circular arc from the robot to the lookahead point:
        // (x^2 + y^2) / (2y), in the robot's local frame (strafe = local x, forward = local y).
        double ppRad;
        if (Math.abs(strafeError) < 0.001) {
            ppRad = 1e6; // effectively straight ahead
        } else {
            ppRad = (strafeError * strafeError + forwardError * forwardError) / (2 * strafeError);
        }

        double speed = 0.7 + 0.3 * Math.min(Math.abs(ppRad), 200) / 200;
        double targetFwd = speed * (reversedSpline ? -1.0 : 1.0);
        double turn = Range.clip(targetFwd * trackWidth / ppRad, -0.9, 0.9);

        // Re-find the closest spline point (not the lookahead point) for cross-track strafe correction.
        int index = splineIndex;
        double prevDist = pose.distanceToPoint(targetPoint);
        double dist = pose.distanceToPoint(spline.points.get(index));
        while (index > 0 && prevDist > dist) {
            index--;
            prevDist = dist;
            dist = pose.distanceToPoint(spline.points.get(index));
        }

        Pose2D closeTarget = spline.points.get(index);
        double xErr = pose.x - closeTarget.x;
        double yErr = pose.y - closeTarget.y;
        double crossTrackError = -xErr * Math.sin(closeTarget.h) + yErr * Math.cos(closeTarget.h);
        double strafe = Range.clip(Math.abs(crossTrackError) > 2 ? crossTrackError * 0.01 : 0, -0.2, 0.2);
        double centripetal = centripetalGain * targetFwd * targetFwd / ppRad;

        setMecanumPowers(targetFwd, turn, centripetal, strafe);
    }

    private void pidToPoint() {
        pidH.updatePID(hP, hI, hD);
        pidStrafe.updatePID(strafeP, strafeI, strafeD);
        pidForward.updatePID(forwardP, forwardI, forwardD);

        double strafe = pidStrafe.update(strafeError);
        double forward = pidForward.update(forwardError);
        double turn = pidH.update(hError);

        double lF = forward - strafe - turn;
        double lB = forward + strafe - turn;
        double rB = forward - strafe + turn;
        double rF = forward + strafe + turn;
        setDrivePower(normalize(lF, rF, lB, rB));
    }

    private void setMecanumPowers(double forward, double turn, double centripetal, double strafe) {
        double lF = forward - turn - centripetal - strafe;
        double lB = forward - turn + centripetal + strafe;
        double rB = forward + turn - centripetal - strafe;
        double rF = forward + turn + centripetal + strafe;
        setDrivePower(normalize(lF, rF, lB, rB));
    }

    private double[] normalize(double lF, double rF, double lB, double rB) {
        double max = Math.max(Math.abs(lF), Math.max(Math.abs(rF), Math.max(Math.abs(lB), Math.abs(rB))));
        if (max > 1.0) {
            lF /= max; rF /= max; lB /= max; rB /= max;
        }
        return new double[]{lF, rF, lB, rB};
    }

    public void setDrivePower(double[] p) {
        setDrivePower(p[0] * maxSpeed, p[1] * maxSpeed, p[2] * maxSpeed, p[3] * maxSpeed);
    }

    public void setDrivePower(double lFPower, double rFPower, double lBPower, double rBPower) {
        frontLeft.setPower(Range.clip(lFPower, -maxSpeed, maxSpeed));
        frontRight.setPower(Range.clip(rFPower, -maxSpeed, maxSpeed));
        backLeft.setPower(Range.clip(lBPower, -maxSpeed, maxSpeed));
        backRight.setPower(Range.clip(rBPower, -maxSpeed, maxSpeed));
    }

    /** Basic robot-centric mecanum drive from gamepad sticks -- for a manual TeleOp. */
    public void drive(Gamepad gamepad) {
        maxSpeed = 1;
        double ly = -gamepad.left_stick_y;
        double ry = -gamepad.right_stick_y;
        double lx = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        setDrivePower(
                ly + ry + rx + lx,
                ly + ry - rx - lx,
                ly + ry + rx - lx,
                ly + ry - rx + lx
        );
    }

    /**
     * Field-centric mecanum drive.
     * @param mirrored flip forward/back field heading, e.g. for a mirrored red/blue alliance start
     */
    public void fieldCentric(Gamepad gamepad, double headingRad, boolean mirrored) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        double h = headingRad + (mirrored ? Math.PI : 0);

        double rotX = x * Math.cos(-h) - y * Math.sin(-h);
        double rotY = x * Math.sin(-h) + y * Math.cos(-h);
        rotX *= 1.1; // counteract strafing being slightly weaker than forward/back

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeft.setPower((rotY + rotX + rx) / denom);
        backLeft.setPower((rotY - rotX + rx) / denom);
        frontRight.setPower((rotY - rotX - rx) / denom);
        backRight.setPower((rotY + rotX - rx) / denom);
    }

    public boolean atPoint() {
        return Math.abs(strafeError) < pointToleranceIn && Math.abs(forwardError) < pointToleranceIn
                && Math.abs(hError) < headingToleranceDeg;
    }

    public boolean atClosePoint() {
        return Math.abs(strafeError) < closePointToleranceIn && Math.abs(forwardError) < closePointToleranceIn
                && Math.abs(hError) < closeHeadingToleranceDeg;
    }

    public boolean isBusy() {
        return driveState != DriveState.PARK && driveState != DriveState.HOLD_POS;
    }

    /** Drive straight to a single point using the PID hold controller (no spline). */
    public void goToPoint(Pose2D targetPose, boolean driveThrough, double maxSpeed) {
        targetPoint = targetPose;
        driveState = DriveState.PID;
        goThroughPoint = driveThrough;
        this.maxSpeed = maxSpeed;
    }

    /** Follow an arbitrary pre-built Spline (see Neuron Path Planner for building one visually). */
    public void followSpline(Spline spline, boolean driveThrough) {
        this.spline = spline;
        driveState = DriveState.SPLINE;
        goThroughPoint = driveThrough;
    }

    /** Convenience: spline directly from the current pose to a single target pose. */
    public void splineToPoint(Pose2D pose, boolean driveThrough, boolean reversed, double factor, double maxSpeed) {
        Spline s = new Spline();
        s.addPoint(this.pose, reversed, 1);
        s.addPoint(pose, reversed, factor);
        reversedSpline = reversed;
        this.maxSpeed = maxSpeed;
        followSpline(s, driveThrough);
    }

    public void relocalize(double x, double y, double hDeg) {
        localizer.setPosition(new Pose2D(x, y, Math.toRadians(hDeg)));
    }
}
