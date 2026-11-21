package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.TeleOpConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.FieldConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.LiftShooter;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.MathFunc;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * HoneyComb match TeleOp, all on gamepad 1:
 *
 *   Left stick       drive and strafe (keeps working while a shot lines up)
 *   Right stick X    turn. While a shot lines up the robot turns itself; moving it cancels the shot
 *   LB (hold)        slow mode
 *   LT (hold)        intake (also cancels a shot that is lining up)
 *   Y (hold)         outtake (also cancels a shot that is lining up)
 *   RT (press)       one-button shot: turns to the goal, sets lift + flywheel from the distance,
 *                    fires when ready, then back to normal (see LiftShooter)
 *   RT again         during a shot that's lining up: fire as soon as the lift is up
 *   Back             reset odometry: the robot must be at the center of the field facing -90°
 *                    (FieldConstants.resetPose)
 *
 * During Init: X = blue alliance, B = red (picks the goal).
 * ShooterConstants.USE_ODOMETRY = false makes RT the old fixed shot (no turning).
 */
@TeleOp(name = "Honeycomb Robot Teleop", group = "Honeycomb")
public class Teleop extends OpMode {
    LiftShooter liftShooter;
    MecanumDrive mecanumDrive;
    NeuronLocalizer localizer;
    private boolean rtWasDown = false;
    // Field debug: what the last Back press did.
    private int resetCount = 0;
    private Pose2D resetSent = null, resetReadBack = null;
    //NOTE FOR FUTURE MICAH, TRACKWIDTH IS 16.1
    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry, gamepad1);
        localizer = LiftShooter.createLocalizer(hardwareMap, telemetry);
        localizer.calibrate(false); // recalibrate the IMU; the pose is set with Back on the reset spot
        liftShooter = new LiftShooter(hardwareMap, telemetry, mecanumDrive, localizer);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) liftShooter.alliance = FieldConstants.Alliance.BLUE;
        if (gamepad1.bWasPressed()) liftShooter.alliance = FieldConstants.Alliance.RED;
        telemetry.addLine("X = blue, B = red");
        telemetry.addData("alliance", liftShooter.alliance);
        telemetry.addData("shot table", liftShooter.shotTable.loadedFromFile ? "loaded from Control Hub" : "code defaults");
        telemetry.addLine("Press Back on the reset spot once the match starts if odometry isn't set yet.");
        telemetry.update();
    }

    @Override
    public void start() {
        liftShooter.intakeLift.moveDown();
    }

    @Override
    public void loop() {
        liftShooter.beginLoop(); // one bulk read per hub + the one Pinpoint read, before anything else

        boolean rtDown = gamepad1.right_trigger >= TRIGGER_THRESHOLD;
        boolean rtPressed = rtDown && !rtWasDown;
        rtWasDown = rtDown;
        boolean intakeHeld = gamepad1.left_trigger >= TRIGGER_THRESHOLD;
        boolean outtakeHeld = gamepad1.y;
        // Only cancel on the right stick when the robot is turning itself (it isn't in the fixed shot).
        boolean turning = ShooterConstants.USE_ODOMETRY && Math.abs(gamepad1.right_stick_x) > STICK_CANCEL_DEADBAND;

        if (gamepad1.backWasPressed()) {
            liftShooter.resetOdometry();
            resetCount++;
            resetSent = FieldConstants.resetPose(liftShooter.alliance);
            resetReadBack = liftShooter.pose; // resetOdometry() re-reads the Pinpoint right after setting it
        }
        boolean cancelHeld = intakeHeld || outtakeHeld || turning;
        if (cancelHeld) liftShooter.cancelShot();
        else if (rtPressed) liftShooter.shootPressed(); // RT does nothing while a cancel input is held
        liftShooter.intakeHeld = intakeHeld;
        liftShooter.outtakeHeld = outtakeHeld;

        liftShooter.update();   // decides the turn override, so it runs before the drive
        mecanumDrive.update();

        liftShooter.addTelemetry(telemetry);
        if (SHOW_FIELD_DEBUG) addFieldDebug();
        telemetry.update();
    }

    /**
     * Everything that decides where the robot aims, in the order the code uses it:
     * Pinpoint health -> pose -> last Back reset -> which side of the CELL switch line -> both
     * CELLs -> the one picked -> heading to it vs the robot's heading.
     */
    private void addFieldDebug() {
        Pose2D p = liftShooter.pose;
        FieldConstants.Alliance a = liftShooter.alliance;
        double dx = p.x - FieldConstants.FIELD_CENTER_X_IN;
        double dy = p.y - FieldConstants.FIELD_CENTER_Y_IN;
        double band = FieldConstants.CELL_SWITCH_BAND_IN;

        telemetry.addLine();
        telemetry.addLine("===== FIELD DEBUG =====");
        telemetry.addData("pinpoint", "%s, %.0f Hz, calibrated %s",
                localizer.pinpoint.getDeviceStatus(), localizer.pinpoint.getFrequency(), localizer.isCalibrated());
        telemetry.addData("pose (odometry)", "x %.1f  y %.1f  h %.1f°", p.x, p.y, Math.toDegrees(p.h));
        telemetry.addData("from field center", "dx %+.1f  dy %+.1f  (center %.0f, %.0f)",
                dx, dy, FieldConstants.FIELD_CENTER_X_IN, FieldConstants.FIELD_CENTER_Y_IN);
        telemetry.addData("velocity", "x %+.1f  y %+.1f in/s   (drive forward/strafe and watch which one moves)",
                localizer.velX(), localizer.velY());

        if (resetCount == 0) {
            telemetry.addData("Back resets", "none yet -- press Back at the center facing %.0f°",
                    FieldConstants.RESET_HEADING_DEG);
        } else {
            telemetry.addData("Back resets", "%d. sent x %.1f y %.1f h %.0f° -> read back x %.1f y %.1f h %.0f°",
                    resetCount, resetSent.x, resetSent.y, Math.toDegrees(resetSent.h),
                    resetReadBack.x, resetReadBack.y, Math.toDegrees(resetReadBack.h));
        }

        String side = dx > band ? "SCORING side (dx > +" + band + ")"
                : dx < -band ? "AUDIENCE side (dx < -" + band + ")"
                : "inside ±" + band + " band: keeps last CELL";
        telemetry.addData("CELL switch", "on X only: %s   (AUTO_CELL %s; y is ignored)",
                side, FieldConstants.AUTO_CELL);

        FieldConstants.Cell cell = FieldConstants.targetCell(p, a);
        Pose2D scoring = FieldConstants.cellPosition(FieldConstants.Cell.SCORING, a);
        Pose2D audience = FieldConstants.cellPosition(FieldConstants.Cell.AUDIENCE, a);
        telemetry.addData("CELLs (" + a + " HIVE)", "SCORING x %.1f y %.1f | AUDIENCE x %.1f y %.1f",
                scoring.x, scoring.y, audience.x, audience.y);
        telemetry.addData("aiming at", "%s CELL, %.1f in away", cell, FieldConstants.distanceToGoal(p, a));

        double want = FieldConstants.headingToGoal(p, a);
        telemetry.addData("heading", "robot %.1f°  wants %.1f°  error %+.1f°  (aim %s)",
                Math.toDegrees(p.h), Math.toDegrees(want), Math.toDegrees(MathFunc.angleWrap(want - p.h)),
                liftShooter.aim.isActive() ? "turning" : "idle");
    }
}
