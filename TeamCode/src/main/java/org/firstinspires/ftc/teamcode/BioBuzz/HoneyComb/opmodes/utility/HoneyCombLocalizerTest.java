package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.utility;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.LiftShooter;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Odometry check. Push the robot around by hand (no motors move here) and confirm:
 *   push forward              -> x goes UP
 *   push left                 -> y goes UP
 *   turn counter-clockwise    -> heading goes UP
 * If one is backwards, fix the pod direction in Constants.OdometryConstants, not the math.
 *
 * Then the two real checks:
 *   1. Push it 48 in forward along a tape measure, then 48 in sideways: x / y should read ~48.
 *   2. Spin it 5 turns in place. While it spins, x and y should stay near 0; if they trace a
 *      circle, the pod offsets (X_POD_OFFSET_MM / Y_POD_OFFSET_MM) or their signs are wrong.
 *
 * A = zero the position. The raw pod counts and the Pinpoint's status show whether a problem is
 * the pods, the wiring or the Pinpoint itself.
 */
@TeleOp(name = "HoneyComb Localizer Test", group = "Honeycomb Tuning")
public class HoneyCombLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        NeuronLocalizer localizer = LiftShooter.createLocalizer(hardwareMap, telemetry);
        localizer.setPosition(new Pose2D(0, 0, 0));
        localizer.calibrate(true);
        GoBildaPinpointDriver pinpoint = localizer.pinpoint;

        while (opModeInInit()) {
            pinpoint.update();
            addPinpointDiagnostics(pinpoint);
            telemetry.addLine("Push the robot now: raw counts should change. Then press Start.");
            telemetry.update();
        }

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) localizer.setPosition(new Pose2D(0, 0, 0));
            Pose2D pose = localizer.getPose();
            telemetry.addData("x (in)", "%.2f", pose.x);
            telemetry.addData("y (in)", "%.2f", pose.y);
            telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.h));
            telemetry.addLine("A = zero");
            telemetry.addLine();
            addPinpointDiagnostics(pinpoint);
            telemetry.update();
        }
    }

    private void addPinpointDiagnostics(GoBildaPinpointDriver pinpoint) {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        telemetry.addData("pinpoint status", status);
        telemetry.addData("raw X pod (forward)", pinpoint.getEncoderX());
        telemetry.addData("raw Y pod (strafe)", pinpoint.getEncoderY());
        telemetry.addData("pinpoint update rate", "%.0f Hz", pinpoint.getFrequency());
        switch (status) {
            case FAULT_X_POD_NOT_DETECTED:
                telemetry.addLine("-> X (forward) pod not detected: check its cable in the Pinpoint's X port");
                break;
            case FAULT_Y_POD_NOT_DETECTED:
                telemetry.addLine("-> Y (strafe) pod not detected: check its cable in the Pinpoint's Y port");
                break;
            case FAULT_NO_PODS_DETECTED:
                telemetry.addLine("-> no pods detected: check both pod cables are in the Pinpoint's X and Y ports");
                break;
            case FAULT_IMU_RUNAWAY:
                telemetry.addLine("-> IMU fault: keep the robot completely still and restart the OpMode");
                break;
            case FAULT_BAD_READ:
                telemetry.addLine("-> bad I2C read: check the Pinpoint's I2C cable and its I2C bus in the config");
                break;
            case NOT_READY:
            case CALIBRATING:
                telemetry.addLine("-> still starting up: keep the robot still for a second");
                break;
            default:
                break;
        }
    }
}
