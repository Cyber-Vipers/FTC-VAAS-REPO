package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.TeleOpConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.FieldConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.LiftShooter;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;

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
 *   RT again         during a shot that's lining up: fire now
 *   Back             reset odometry: the robot must be on the alliance's reset spot
 *                    (FieldConstants.resetPose)
 *
 * During Init: X = blue alliance, B = red (picks the goal and the reset spot).
 * ShooterConstants.USE_ODOMETRY = false makes RT the old fixed shot (no turning).
 */
@TeleOp(name = "Honeycomb Robot Teleop", group = "Honeycomb")
public class Teleop extends OpMode {
    LiftShooter liftShooter;
    MecanumDrive mecanumDrive;
    NeuronLocalizer localizer;
    private boolean rtWasDown = false;

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

        if (gamepad1.backWasPressed()) liftShooter.resetOdometry();
        if (intakeHeld || outtakeHeld || turning) liftShooter.cancelShot();
        if (rtPressed) liftShooter.shootPressed();
        liftShooter.intakeHeld = intakeHeld;
        liftShooter.outtakeHeld = outtakeHeld;

        liftShooter.update();   // decides the turn override, so it runs before the drive
        mecanumDrive.update();

        liftShooter.addTelemetry(telemetry);
        telemetry.update();
    }
}
