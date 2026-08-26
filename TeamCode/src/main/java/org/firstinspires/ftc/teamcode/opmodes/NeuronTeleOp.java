package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.ExampleClaw;

/**
 * Manual TeleOp on top of Neuron Pathing. Drive with the left stick's y/x and
 * rotate with the right stick's x -- robot-centric, no field-oriented math, so
 * it's a good first sanity check that your motors/directions are wired
 * correctly before layering in autonomous paths.
 *
 * Also shows the subsystem pattern: ExampleClaw is constructed once alongside
 * drive/localizer, driven by a gamepad button, and updated every loop just
 * like drive.update(). See docs/subsystems.md for the full writeup -- swap
 * ExampleClaw for your own game's mechanisms following the same shape.
 *
 * TODO: replace the xOffsetMm/yOffsetMm placeholders with your robot's actual
 * Pinpoint pod offsets (see the Installation doc / Tuning guide).
 */
@TeleOp(name = "Neuron TeleOp (Basic)")
public class NeuronTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        NeuronLocalizer localizer = new NeuronLocalizer(hardwareMap, telemetry, /*xOffsetMm=*/0, /*yOffsetMm=*/0);
        NeuronDrive drive = new NeuronDrive(hardwareMap, localizer, /*trackWidthIn=*/14);
        ExampleClaw claw = new ExampleClaw(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(gamepad1);
            drive.update();

            // A subsystem's public methods are called directly, gated by gamepad buttons.
            // *WasPressed() edge-detects so holding the button doesn't spam toggle().
            if (gamepad1.aWasPressed()) claw.toggle();
            claw.update();

            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.h));
            telemetry.addData("claw", claw.state);
            telemetry.update();
        }
    }
}
