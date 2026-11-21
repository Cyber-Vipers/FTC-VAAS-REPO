package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.ShooterConstants.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.LiftConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.OdometryConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.FieldConstants;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

import java.util.ArrayList;
import java.util.List;

/**
 * HoneyComb's lift shooter with the one-button shot. One RT press runs the whole shot:
 *
 *   NORMAL   same as before: LT intakes, Y outtakes, lift down, flywheel spinning in
 *   SPIN_UP  flywheel to the shot table's speed; lift stays down, transfer holds the balls
 *   AIM      lift to the shot table's position, robot turns itself to the goal (left stick
 *            still drives); fires once aimed + flywheel at speed + lift there
 *   FIRE     transfer feeds the balls into the flywheel for FIRE_TIME_MS, still aiming,
 *            then back to NORMAL
 *
 * RT again while lining up fires as soon as the lift is up, without waiting for aim or the
 * flywheel. FIRE always ends after FIRE_MAX_MS. LT, Y,
 * the right stick or SHOT_TIMEOUT_MS cancel back to NORMAL (the TeleOp decides which inputs
 * cancel). With ShooterConstants.USE_ODOMETRY false, the same states run the old fixed shot
 * with no turning. All numbers are in Constants.
 *
 * Every loop: beginLoop() (one bulk read per hub, loop time, the ONE Pinpoint read), then the
 * TeleOp's requests, then update(), which decides what each part does and sends it once.
 */
public class LiftShooter implements Subsystem {
    public enum ShotState { NORMAL, SPIN_UP, AIM, FIRE }

    public final PIDControlledLift lift;
    public final Flywheel flywheel;
    public final ToggledMotor intake, liftTransfer;
    public final toggledServo intakeLift;
    public final MecanumDrive drive;
    public final NeuronLocalizer localizer;
    public final GoalAim aim;
    public ShotTable shotTable;
    private final List<LynxModule> hubs;
    private NeuronDrive neuronDrive;
    public FieldConstants.Alliance alliance = FieldConstants.Alliance.RED;
    public ShotState state = ShotState.NORMAL;

    /** Driver inputs for NORMAL, set by the TeleOp every loop. */
    public boolean intakeHeld = false, outtakeHeld = false;

    // This loop's values, kept for telemetry.
    public Pose2D pose = new Pose2D(0, 0, 0);
    public double loopMs = 0, distanceToGoal = 0, liftTarget = 0, speedTarget = 0;
    public boolean aimed = false;
    /** What a shot that's lining up is still waiting for, or how the last one ended. */
    public String shotStatus = "";

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime fireTimer = new ElapsedTime();
    private double firedMs = 0;
    private boolean fireNow = false;

    public LiftShooter(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive drive, NeuronLocalizer localizer) {
        // MANUAL bulk caching: beginLoop() clears the cache once per loop, so each hub does ONE
        // bulk read per loop however often a position or velocity is read. (AUTO starts a new
        // bulk read whenever the same value is read twice, which happens all the time here.)
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        this.drive = drive;
        this.localizer = localizer;
        aim = new GoalAim(drive, localizer);
        flywheel = new Flywheel(hardwareMap, telemetry, FlywheelConstants.PIDF, FlywheelConstants.NAME,
                FlywheelConstants.AT_SPEED_TOLERANCE_TPS);
        intake = new ToggledMotor(hardwareMap, telemetry, IntakeConstants.INTAKE_NAME);
        liftTransfer = new ToggledMotor(hardwareMap, telemetry, IntakeConstants.TRANSFER_NAME);
        lift = new PIDControlledLift(hardwareMap, telemetry, LiftConstants.TOLERANCE_TICKS,
                LiftConstants.HIGHEST_TICKS, LiftConstants.LOWEST_TICKS, LiftConstants.REVERSED, LiftConstants.PIDF);
        lift.resetEncoder(); // assumes it's down; if not, it re-zeroes once it has come to rest at the bottom
        intakeLift = new toggledServo(hardwareMap, telemetry, IntakeConstants.INTAKE_LIFT_UP,
                IntakeConstants.INTAKE_LIFT_DOWN, IntakeConstants.INTAKE_LIFT_NAME, true, true);
        shotTable = ShotTable.load();
    }


    public LiftShooter(HardwareMap hardwareMap, Telemetry telemetry, NeuronDrive drive, NeuronLocalizer localizer) {
        // MANUAL bulk caching: beginLoop() clears the cache once per loop, so each hub does ONE
        // bulk read per loop however often a position or velocity is read. (AUTO starts a new
        // bulk read whenever the same value is read twice, which happens all the time here.)
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        this.neuronDrive = drive;
        this.localizer = localizer;
        this.drive = new MecanumDrive(hardwareMap,telemetry,new Gamepad(),true);
        aim = new GoalAim(this.drive, localizer);
        flywheel = new Flywheel(hardwareMap, telemetry, FlywheelConstants.PIDF, FlywheelConstants.NAME,
                FlywheelConstants.AT_SPEED_TOLERANCE_TPS);
        intake = new ToggledMotor(hardwareMap, telemetry, IntakeConstants.INTAKE_NAME);
        liftTransfer = new ToggledMotor(hardwareMap, telemetry, IntakeConstants.TRANSFER_NAME);
        lift = new PIDControlledLift(hardwareMap, telemetry, LiftConstants.TOLERANCE_TICKS,
                LiftConstants.HIGHEST_TICKS, LiftConstants.LOWEST_TICKS, LiftConstants.REVERSED, LiftConstants.PIDF);
        lift.resetEncoder(); // assumes it's down; if not, it re-zeroes once it has come to rest at the bottom
        intakeLift = new toggledServo(hardwareMap, telemetry, IntakeConstants.INTAKE_LIFT_UP,
                IntakeConstants.INTAKE_LIFT_DOWN, IntakeConstants.INTAKE_LIFT_NAME, true, true);
        shotTable = ShotTable.load();
    }
    /** The Pinpoint localizer with the pod settings from Constants.OdometryConstants. */
    public static NeuronLocalizer createLocalizer(HardwareMap hardwareMap, Telemetry telemetry) {
        return new NeuronLocalizer(hardwareMap, telemetry, OdometryConstants.NAME,
                OdometryConstants.X_POD_OFFSET_MM, OdometryConstants.Y_POD_OFFSET_MM,
                OdometryConstants.POD_TYPE, OdometryConstants.X_POD_DIRECTION, OdometryConstants.Y_POD_DIRECTION);
    }

    /** First thing every loop: fresh bulk read, loop time, and this loop's one Pinpoint read. */
    public Pose2D beginLoop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        pose = localizer.getPose(); // reads the Pinpoint over I2C: only here, once per loop
        return pose;
    }

    // ---- requests ------------------------------------------------------------------------

    /** RT: start a shot; while one is lining up, fire right away. */
    public void shootPressed() {
        switch (state) {
            case NORMAL:
                state = ShotState.SPIN_UP;
                shotTimer.reset();
                fireNow = false;
                shotStatus = "spinning up";
                break;
            case SPIN_UP:
                fireNow = true; // skip the lead; fires as soon as the lift is up
                state = ShotState.AIM;
                break;
            case AIM:
                fireNow = true; // don't wait for aim or flywheel; still wait for the lift to be up
                break;
            default:
                break;
        }
    }

    /** Cancel a shot that hasn't fired yet. */
    public void cancelShot() {
        if (isLiningUp()) {
            shotStatus = "cancelled";
            toNormal();
        }
    }

    public boolean isLiningUp() {
        return state == ShotState.SPIN_UP || state == ShotState.AIM;
    }

    public boolean isShooting() {
        return state != ShotState.NORMAL;
    }

    /** Robot must be sitting on the alliance's reset spot (FieldConstants.resetPose). */
    public void resetOdometry() {
        localizer.setPosition(FieldConstants.resetPose(alliance));
        pose = localizer.getPose(); // otherwise the rest of this loop aims from the pre-reset pose
    }

    // ---- loop ----------------------------------------------------------------------------

    @Override
    public void update() {
        distanceToGoal = FieldConstants.distanceToGoal(pose, alliance);
        liftTarget = USE_ODOMETRY ? shotTable.liftTicks(distanceToGoal) : FIXED_LIFT_TICKS;
        speedTarget = USE_ODOMETRY ? shotTable.flywheelTps(distanceToGoal) : FIXED_FLYWHEEL_TPS;

        if (isLiningUp() && shotTimer.milliseconds() > SHOT_TIMEOUT_MS) {
            shotStatus = "timed out, " + shotStatus;
            toNormal();
        }

        switch (state) {
            case NORMAL:
                normal();
                break;

            case SPIN_UP:
                flywheel.setTarget((int) Math.round(speedTarget));
                lift.bottomOut();
                holdBalls();
                if (flywheel.isUp2Speed() || shotTimer.milliseconds() > SPINUP_LEAD_MS) state = ShotState.AIM;
                break;

            case AIM:
            case FIRE:
                // Recalculated every loop, so the targets follow the robot if the driver is still moving.
                flywheel.setTarget((int) Math.round(speedTarget));
                lift.setTargetPosition((int) Math.round(liftTarget));
                aimed = !USE_ODOMETRY || aim.update(pose, FieldConstants.headingToGoal(pose, alliance));
                if (state == ShotState.AIM) {
                    holdBalls();
                    boolean atSpeed = flywheel.isUp2Speed();
                    boolean liftThere = lift.atTarget();
                    boolean slowEnough = FIRE_MAX_SPEED_IN_S <= 0
                            || Math.hypot(localizer.velX(), localizer.velY()) < FIRE_MAX_SPEED_IN_S;
                    shotStatus = waitingOn(aimed, atSpeed, liftThere, slowEnough);
                    if (fireNow ? liftThere : aimed && atSpeed && liftThere && slowEnough) startFiring();
                } else {
                    feed();
                    // FIRE_MAX_MS ends it even if FIRE_RECOVER_TPS kept pausing the feed (weak battery).
                    if (firedMs > FIRE_TIME_MS || fireTimer.milliseconds() > FIRE_MAX_MS) {
                        shotStatus = "fired";
                        toNormal();
                        normal();
                    }
                }
                break;
        }
        updateHardware();
    }

    /** Same as the old TeleOp when not shooting. */
    private void normal() {
        aimed = false;
        lift.bottomOut();
        flywheel.setTarget(-FlywheelConstants.INTAKE_TPS); // spinning in
        if (intakeHeld) {
            intake.setReverse();
            intake.On();
            liftTransfer.setReverse();
            liftTransfer.On();
        } else if (outtakeHeld) {
            intake.setForward();
            intake.On();
            liftTransfer.Off();
        } else {
            intake.Off();
            liftTransfer.Off();
        }
        if (lift.isDown() && flywheel.isUp2Speed()) intakeLift.moveDown();
    }

    /** Lining up: the transfer holds the balls back while the intake keeps pulling in (as the old shot did). */
    private void holdBalls() {
        liftTransfer.Off();
        intake.setReverse();
        intake.On();
    }

    /** Firing: the transfer pushes the balls into the flywheel; FIRE_TIME_MS counts only time spent feeding. */
    private void feed() {
        double dt = feedTimer.milliseconds();
        feedTimer.reset();
        intake.setReverse();
        intake.On();
        liftTransfer.setForward();
        boolean recovering = FIRE_RECOVER_TPS > 0
                && flywheel.getVelocity() < flywheel.getTargetVelocity() - FIRE_RECOVER_TPS;
        if (recovering) {
            liftTransfer.Off(); // let the flywheel get back up to speed before the next ball
        } else {
            liftTransfer.On();
            firedMs += dt;
        }
    }

    private void startFiring() {
        state = ShotState.FIRE;
        firedMs = 0;
        feedTimer.reset();
        fireTimer.reset();
        shotStatus = fireNow ? "firing (fire now)" : "firing";
    }

    private void toNormal() {
        state = ShotState.NORMAL;
        fireNow = false;
        aim.stop();
    }

    private String waitingOn(boolean aimed, boolean atSpeed, boolean liftThere, boolean slowEnough) {
        List<String> missing = new ArrayList<>();
        if (!aimed) missing.add(String.format("aim %.1f°", aim.errorDeg));
        if (!atSpeed) missing.add("flywheel");
        if (!liftThere) missing.add("lift");
        if (!slowEnough) missing.add("robot moving");
        return missing.isEmpty() ? "ready" : "waiting on: " + String.join(", ", missing);
    }

    /** Send this loop's outputs. update() calls it; tuners that drive the parts directly call it themselves. */
    public void updateHardware() {
        lift.update();
        flywheel.update();
        liftTransfer.update();
        intake.update();
        intakeLift.update();
        liftTransfer.voltageControl();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("loop", "%.0f ms", loopMs);
        telemetry.addData("shot", "%s%s", state, shotStatus.isEmpty() ? "" : "  (" + shotStatus + ")");
        telemetry.addData("alliance", "%s%s", alliance, USE_ODOMETRY ? "" : "   USE_ODOMETRY off: fixed shot");
        telemetry.addData("pose", "x %.1f  y %.1f  h %.1f°", pose.x, pose.y, Math.toDegrees(pose.h));
        telemetry.addData("to goal", "%.1f in  (%s CELL)", distanceToGoal, FieldConstants.targetCell(pose, alliance));
        telemetry.addData("shot from here", "lift %.0f, flywheel %.0f  (table: %s)", liftTarget, speedTarget,
                shotTable.loadedFromFile ? "from Control Hub" : "code defaults");
        lift.addTelemetry(telemetry);
        flywheel.addTelemetry(telemetry);
    }
}
