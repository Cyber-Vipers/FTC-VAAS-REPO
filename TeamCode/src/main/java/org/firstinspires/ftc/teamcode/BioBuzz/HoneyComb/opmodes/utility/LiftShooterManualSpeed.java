package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.teleop.LiftShooterTeleop;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Lift;

/*
 * Fully manual version of LiftShooter. Nothing happens on its own; every motor is on its own button.
 *
 * FLYWHEEL
 *   A            - flywheel on/off
 *   dpad up/down - flywheel speed up/down (ticks/s)
 *   right bumper - flip flywheel direction
 *
 * INTAKE
 *   B               - intake on/off
 *   X               - flip intake direction
 *   dpad right/left - intake power up/down (0 to 1)
 *
 * LIFT
 *   hold right trigger - lift up
 *   hold left trigger  - lift down
 *
 * left bumper - switch between coarse and fine step sizes
 */
@Utility(name="Honeycomb Lift Shooter (Manual)")
public class LiftShooterManualSpeed extends OpMode {
    Lift lift;
    Flywheel flywheel;
    DcMotorEx intake;

    // Starting point only. Tune P and F for your flywheel.
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10, 3, 0, 13);

    public int highestLift = 1062;
    public int lowestLift = 0;
    public int globalTolerance = 20;
    public int liftStep = 40; // ticks per loop while a trigger is held

    public boolean flywheelOn = false;
    public int flywheelSpeed = 1500;
    public int flywheelStepCoarse = 100;
    public int flywheelStepFine = 10;

    public boolean intakeOn = false;
    public boolean intakeReversed = false;
    public double intakePower = 1.0;
    public double intakeStepCoarse = 0.1;
    public double intakeStepFine = 0.01;

    public boolean fineMode = false;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap, telemetry, pidfCoefficients, 40);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift = new Lift(hardwareMap, telemetry, globalTolerance, highestLift, lowestLift, false, LiftShooterTeleop.liftPidf);
        lift.resetEncoder();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Lift Position", lift.getPosition());
    }

    @Override
    public void loop() {
        if (gamepad1.leftBumperWasPressed()) {
            fineMode = !fineMode;
        }
        flywheelControls();
        intakeControls();
        liftControls();

        telemetry.addData("Step size", fineMode ? "FINE" : "COARSE");
        telemetry.addLine();
        telemetry.addData("Flywheel", (flywheelOn ? "ON" : "OFF") + (flywheel.isReversed() ? " (REVERSED)" : ""));
        telemetry.addData("  Speed setting (ticks/s)", flywheelSpeed);
        telemetry.addData("  Actual (ticks/s)", flywheel.getVelocity());
        telemetry.addData("  Is up to speed", flywheelOn && flywheel.isUp2Speed());
        telemetry.addLine();
        telemetry.addData("Intake", (intakeOn ? "ON" : "OFF") + (intakeReversed ? " (REVERSED)" : ""));
        telemetry.addData("  Power", "%.2f", intakePower);
        telemetry.addLine();
        telemetry.addData("Lift Position", lift.getPosition());
        telemetry.addData("Lift Target", lift.motorEx.getTargetPosition());
    }

    public void flywheelControls() {
        int step = fineMode ? flywheelStepFine : flywheelStepCoarse;
        if (gamepad1.aWasPressed()) {
            flywheelOn = !flywheelOn;
        }
        if (gamepad1.dpadUpWasPressed()) {
            flywheelSpeed += step;
        }
        if (gamepad1.dpadDownWasPressed()) {
            flywheelSpeed = Math.max(0, flywheelSpeed - step);
        }
        if (gamepad1.rightBumperWasPressed()) {
            flywheel.toggleDirection();
        }
        flywheel.setSpeed(flywheelOn ? flywheelSpeed : 0);
    }

    public void intakeControls() {
        double step = fineMode ? intakeStepFine : intakeStepCoarse;
        if (gamepad1.bWasPressed()) {
            intakeOn = !intakeOn;
        }
        if (gamepad1.xWasPressed()) {
            intakeReversed = !intakeReversed;
        }
        if (gamepad1.dpadRightWasPressed()) {
            intakePower += step;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            intakePower -= step;
        }
        intakePower = Range.clip(intakePower, 0, 1);
        double power = intakeOn ? intakePower : 0;
        intake.setPower(intakeReversed ? -power : power);
    }

    public void liftControls() {
        int position = (int) lift.getPosition();
        if (gamepad1.right_trigger > 0.5) {
            lift.goToPosition(position + liftStep);
        } else if (gamepad1.left_trigger > 0.5) {
            lift.goToPosition(Math.max(lowestLift, position - liftStep));
        }
    }
}
