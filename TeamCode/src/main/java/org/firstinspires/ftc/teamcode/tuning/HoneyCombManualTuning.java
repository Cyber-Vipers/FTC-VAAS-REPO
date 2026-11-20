package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.teleop.LiftShooterTeleop;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Lift;

/*
 * HoneyComb manual tuning (all on gamepad1).
 *
 * FLYWHEEL
 *   A                  - toggle flywheel on/off
 *   dpad up/down       - raise/lower flywheel speed
 *   right stick button - switch between VELOCITY mode (target in ticks/sec, hub holds the speed)
 *                        and POWER mode (raw motor power 0 to 1, no speed control)
 *
 * TRANSFER (motor power, 0 to 1)
 *   B               - toggle transfer on/off
 *   X               - reverse transfer direction
 *   dpad right/left - raise/lower transfer power
 *
 * ARM LIFT (HoneyComb Lift subsystem, encoder ticks)
 *   Y                 - toggle lift between down and the "up" target
 *   right/left bumper - raise/lower the "up" target (lift follows live if it's up)
 *   back              - zero the lift encoder at its current spot
 *
 * left stick button - switch between coarse and fine step sizes
 */
@TeleOp(name = "HoneyComb Manual Tuning", group = "Tuning")
public class HoneyCombManualTuning extends LinearOpMode {
    // Change these to match the names in your robot configuration
    private static final String FLYWHEEL_NAME = "flywheel";
    private static final String TRANSFER_NAME = "intake";

    private static final double FLYWHEEL_VELOCITY_STEP_COARSE = 100;
    private static final double FLYWHEEL_VELOCITY_STEP_FINE = 10;
    private static final double FLYWHEEL_POWER_STEP_COARSE = 0.1;
    private static final double FLYWHEEL_POWER_STEP_FINE = 0.01;
    private static final double TRANSFER_STEP_COARSE = 0.1;
    private static final double TRANSFER_STEP_FINE = 0.01;
    private static final int LIFT_STEP_COARSE = 50;
    private static final int LIFT_STEP_FINE = 5;

    private static final int LIFT_TOLERANCE = 40;
    private static final int LIFT_DOWN_POSITION = 0;
    private static final double LIFT_POWER = 0.8;

    private DcMotorEx flywheel;
    private DcMotorEx transfer;
    private Lift lift;

    private boolean flywheelOn = false;
    private boolean flywheelPowerMode = false;
    private double flywheelVelocityTarget = 1200; // ticks/sec
    private double flywheelPowerTarget = 0.5;

    private double transferPower = 0.5;
    private boolean transferOn = false;
    private boolean transferReversed = false;

    private int liftUpTarget = 600; // ticks
    private boolean liftUp = false;

    private boolean fineMode = false;

    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();
    PIDFCoefficients pidf = new PIDFCoefficients(0, 0, 0, 0);

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        transfer = hardwareMap.get(DcMotorEx.class, TRANSFER_NAME);
        lift = new Lift(hardwareMap, telemetry, LIFT_TOLERANCE, liftUpTarget, LIFT_DOWN_POSITION, false, LiftShooterTeleop.liftPidf);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setTargetPosition(LIFT_DOWN_POSITION);

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            previous.copy(current);
            current.copy(gamepad1);

            if (pressed(current.left_stick_button, previous.left_stick_button)) {
                fineMode = !fineMode;
            }
            double transferStep = fineMode ? TRANSFER_STEP_FINE : TRANSFER_STEP_COARSE;
            int liftStep = fineMode ? LIFT_STEP_FINE : LIFT_STEP_COARSE;

            // Flywheel
            if (pressed(current.a, previous.a)) flywheelOn = !flywheelOn;
            if (pressed(current.right_stick_button, previous.right_stick_button)) {
                flywheelPowerMode = !flywheelPowerMode;
                flywheel.setMode(flywheelPowerMode
                        ? DcMotor.RunMode.RUN_WITHOUT_ENCODER
                        : DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (flywheelPowerMode) {
                double step = fineMode ? FLYWHEEL_POWER_STEP_FINE : FLYWHEEL_POWER_STEP_COARSE;
                if (pressed(current.dpad_up, previous.dpad_up)) flywheelPowerTarget += step;
                if (pressed(current.dpad_down, previous.dpad_down)) flywheelPowerTarget -= step;
                flywheelPowerTarget = Range.clip(flywheelPowerTarget, 0, 1);
                flywheel.setPower(flywheelOn ? flywheelPowerTarget : 0);
            } else {
                double step = fineMode ? FLYWHEEL_VELOCITY_STEP_FINE : FLYWHEEL_VELOCITY_STEP_COARSE;
                if (pressed(current.dpad_up, previous.dpad_up)) flywheelVelocityTarget += step;
                if (pressed(current.dpad_down, previous.dpad_down)) flywheelVelocityTarget -= step;
                flywheelVelocityTarget = Math.max(0, flywheelVelocityTarget);
                flywheel.setVelocity(flywheelOn ? flywheelVelocityTarget : 0);
            }

            // Transfer
            if (pressed(current.b, previous.b)) transferOn = !transferOn;
            if (pressed(current.x, previous.x)) transferReversed = !transferReversed;
            if (pressed(current.dpad_right, previous.dpad_right)) transferPower += transferStep;
            if (pressed(current.dpad_left, previous.dpad_left)) transferPower -= transferStep;
            transferPower = Range.clip(transferPower, 0, 1);
            double transferOut = transferOn ? transferPower : 0;
            transfer.setPower(transferReversed ? -transferOut : transferOut);

            // Arm lift
            if (pressed(current.y, previous.y)) liftUp = !liftUp;
            if (pressed(current.right_bumper, previous.right_bumper)) liftUpTarget += liftStep;
            if (pressed(current.left_bumper, previous.left_bumper)) liftUpTarget -= liftStep;
            liftUpTarget = Math.max(LIFT_DOWN_POSITION, liftUpTarget);
            if (pressed(current.back, previous.back)) {
                lift.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftUp = false;
            }
            lift.setTargetPosition(liftUp ? liftUpTarget : LIFT_DOWN_POSITION);
            // Lift.setTargetPosition never sets power, so drive it here
            lift.motorEx.setPower(LIFT_POWER);

            telemetry.addData("Step size", fineMode ? "FINE" : "COARSE");
            telemetry.addLine();
            telemetry.addData("Flywheel", (flywheelOn ? "ON" : "OFF") + (flywheelPowerMode ? " - POWER mode" : " - VELOCITY mode"));
            if (flywheelPowerMode) {
                telemetry.addData("  Power target", "%.2f", flywheelPowerTarget);
            } else {
                telemetry.addData("  Velocity target (ticks/s)", "%.0f", flywheelVelocityTarget);
            }
            telemetry.addData("  Actual (ticks/s)", "%.0f", flywheel.getVelocity());
            telemetry.addData("  Power the hub is sending", "%.2f", flywheel.getPower());
            telemetry.addData("  Current (amps)", "%.2f", flywheel.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("  Encoder position", flywheel.getCurrentPosition());
            telemetry.addData("  Motor type in config", flywheel.getMotorType().getName());
            PIDFCoefficients pidf = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("  Velocity PIDF", "P %.2f  I %.2f  D %.2f  F %.2f", pidf.p, pidf.i, pidf.d, pidf.f);
            telemetry.addLine();
            telemetry.addData("Transfer", (transferOn ? "ON" : "OFF") + (transferReversed ? " (REVERSED)" : ""));
            telemetry.addData("  Power", "%.2f", transferPower);
            telemetry.addLine();
            telemetry.addData("Lift", liftUp ? "UP" : "DOWN");
            telemetry.addData("  Up target (ticks)", liftUpTarget);
            lift.update();
            telemetry.update();
        }

        flywheel.setPower(0);
        transfer.setPower(0);
        lift.motorEx.setPower(0);
    }

    private boolean pressed(boolean now, boolean before) {
        return now && !before;
    }
}
