package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/*
 * Manual tuning opmode for the flywheel, transfer and arm (all on gamepad1).
 *
 * FLYWHEEL (velocity in encoder ticks per second)
 *   A            - toggle flywheel on/off
 *   dpad up/down - raise/lower target velocity
 *
 * TRANSFER (motor power, -1 to 1)
 *   B              - toggle transfer on/off
 *   X              - reverse transfer direction
 *   dpad right/left - raise/lower transfer power
 *
 * ARM (encoder ticks from where it was at init)
 *   Y                  - toggle arm between home (0) and the "up" target
 *   right/left bumper  - raise/lower the "up" target (arm follows live if it's up)
 *   back               - reset arm encoder to 0 at its current spot
 *
 * left stick button - switch between coarse and fine step sizes
 */
@TeleOp(name = "Manual Tuning", group = "Tuning")
public class ManualTuning extends LinearOpMode {
    // Change these to match the names in your robot configuration
    private static final String FLYWHEEL_NAME = "flywheel";
    private static final String TRANSFER_NAME = "transfer";
    private static final String ARM_NAME = "arm";

    private static final double FLYWHEEL_STEP_COARSE = 100;
    private static final double FLYWHEEL_STEP_FINE = 10;
    private static final double TRANSFER_STEP_COARSE = 0.1;
    private static final double TRANSFER_STEP_FINE = 0.01;
    private static final int ARM_STEP_COARSE = 50;
    private static final int ARM_STEP_FINE = 5;
    private static final double ARM_POWER = 0.8;

    private DcMotorEx flywheel;
    private DcMotorEx transfer;
    private DcMotorEx arm;

    private double flywheelTarget = 1500; // ticks/sec
    private boolean flywheelOn = false;

    private double transferPower = 0.5;
    private boolean transferOn = false;
    private boolean transferReversed = false;

    private int armUpTarget = 300; // ticks
    private boolean armUp = false;

    private boolean fineMode = false;

    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        transfer = hardwareMap.get(DcMotorEx.class, TRANSFER_NAME);
        arm = hardwareMap.get(DcMotorEx.class, ARM_NAME);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Ready. Arm encoder zeroed at current position.");
        telemetry.update();

        waitForStart();

        arm.setPower(ARM_POWER);

        while (opModeIsActive()) {
            previous.copy(current);
            current.copy(gamepad1);

            if (pressed(current.left_stick_button, previous.left_stick_button)) {
                fineMode = !fineMode;
            }
            double flywheelStep = fineMode ? FLYWHEEL_STEP_FINE : FLYWHEEL_STEP_COARSE;
            double transferStep = fineMode ? TRANSFER_STEP_FINE : TRANSFER_STEP_COARSE;
            int armStep = fineMode ? ARM_STEP_FINE : ARM_STEP_COARSE;

            // Flywheel
            if (pressed(current.a, previous.a)) flywheelOn = !flywheelOn;
            if (pressed(current.dpad_up, previous.dpad_up)) flywheelTarget += flywheelStep;
            if (pressed(current.dpad_down, previous.dpad_down)) flywheelTarget -= flywheelStep;
            flywheelTarget = Math.max(0, flywheelTarget);
            flywheel.setVelocity(flywheelOn ? flywheelTarget : 0);

            // Transfer
            if (pressed(current.b, previous.b)) transferOn = !transferOn;
            if (pressed(current.x, previous.x)) transferReversed = !transferReversed;
            if (pressed(current.dpad_right, previous.dpad_right)) transferPower += transferStep;
            if (pressed(current.dpad_left, previous.dpad_left)) transferPower -= transferStep;
            transferPower = Range.clip(transferPower, 0, 1);
            double transferOut = transferOn ? transferPower : 0;
            transfer.setPower(transferReversed ? -transferOut : transferOut);

            // Arm
            if (pressed(current.y, previous.y)) armUp = !armUp;
            if (pressed(current.right_bumper, previous.right_bumper)) armUpTarget += armStep;
            if (pressed(current.left_bumper, previous.left_bumper)) armUpTarget -= armStep;
            if (pressed(current.back, previous.back)) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(ARM_POWER);
                armUp = false;
            }
            arm.setTargetPosition(armUp ? armUpTarget : 0);

            telemetry.addData("Step size", fineMode ? "FINE" : "COARSE");
            telemetry.addLine();
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.addData("  Target (ticks/s)", "%.0f", flywheelTarget);
            telemetry.addData("  Actual (ticks/s)", "%.0f", flywheel.getVelocity());
            telemetry.addLine();
            telemetry.addData("Transfer", (transferOn ? "ON" : "OFF") + (transferReversed ? " (REVERSED)" : ""));
            telemetry.addData("  Power", "%.2f", transferPower);
            telemetry.addLine();
            telemetry.addData("Arm", armUp ? "UP" : "HOME");
            telemetry.addData("  Up target (ticks)", armUpTarget);
            telemetry.addData("  Current (ticks)", arm.getCurrentPosition());
            telemetry.update();
        }

        flywheel.setVelocity(0);
        transfer.setPower(0);
        arm.setPower(0);
    }

    private boolean pressed(boolean now, boolean before) {
        return now && !before;
    }
}
