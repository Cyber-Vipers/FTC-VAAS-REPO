package org.firstinspires.ftc.teamcode.neuronpathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Wraps the goBILDA Pinpoint odometry computer and exposes it as a Neuron Pathing
 * Pose2D source. The Pinpoint driver ships with the FTC SDK itself (Hardware
 * module), so no extra dependency is needed beyond the stock FtcRobotController.
 *
 * Configure xOffset/yOffset/podType/encoderDirections for your robot in the
 * constructor call in your OpMode or drive class -- these are NOT hardcoded
 * here, unlike a from-scratch Pinpoint wrapper, since every robot's pod
 * placement is different.
 */
public class NeuronLocalizer {
    private final Telemetry telemetry;
    public final GoBildaPinpointDriver pinpoint;
    private boolean calibrated = false;

    public NeuronLocalizer(HardwareMap hwMap, Telemetry telemetry, String deviceName,
                            double xOffsetMm, double yOffsetMm,
                            GoBildaPinpointDriver.GoBildaOdometryPods podType,
                            GoBildaPinpointDriver.EncoderDirection xEncoderDirection,
                            GoBildaPinpointDriver.EncoderDirection yEncoderDirection) {
        this.telemetry = telemetry;
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, deviceName);

        pinpoint.setOffsets(xOffsetMm, yOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(podType);
        pinpoint.setEncoderDirections(xEncoderDirection, yEncoderDirection);
    }

    /** Convenience constructor using the goBILDA 4-bar pod and the "pinpoint" device name. */
    public NeuronLocalizer(HardwareMap hwMap, Telemetry telemetry, double xOffsetMm, double yOffsetMm) {
        this(hwMap, telemetry, "pinpoint", xOffsetMm, yOffsetMm,
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    public Pose2D getPose() {
        warnIfNotCalibrated();
        pinpoint.update();
        return new Pose2D(
                pinpoint.getPosition().getX(DistanceUnit.INCH),
                pinpoint.getPosition().getY(DistanceUnit.INCH),
                pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
    }

    public double getAngularVelocityDegPerSec() {
        warnIfNotCalibrated();
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    public double velX() {
        warnIfNotCalibrated();
        return pinpoint.getVelX(DistanceUnit.INCH);
    }

    public double velY() {
        warnIfNotCalibrated();
        return pinpoint.getVelY(DistanceUnit.INCH);
    }

    public void setPosition(Pose2D pos) {
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D sdkPose =
                new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                        DistanceUnit.INCH, pos.x, pos.y, AngleUnit.RADIANS, pos.h);
        pinpoint.setPosition(sdkPose);
        pinpoint.update();
    }

    /**
     * Resets position/IMU (resetPos = true) or just recalibrates the IMU, then
     * blocks (briefly) until the Pinpoint reports READY. Call this once during
     * init, after setPosition() with your starting auto pose.
     */
    public void calibrate(boolean resetPos) {
        if (resetPos) {
            pinpoint.resetPosAndIMU();
        } else {
            pinpoint.recalibrateIMU();
        }

        for (int i = 0; i < 20; i++) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            pinpoint.update();
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                calibrated = true;
                if (telemetry != null) {
                    telemetry.log().add("Neuron Pathing: Pinpoint calibrated and position reset");
                }
                return;
            }
        }

        if (telemetry != null) {
            telemetry.log().add("Neuron Pathing: Pinpoint failed to reach READY. Status: "
                    + pinpoint.getDeviceStatus());
        }
    }

    private void warnIfNotCalibrated() {
        if (!calibrated && telemetry != null) {
            telemetry.log().add("Neuron Pathing: reading Pinpoint pose before calibrate() was called");
        }
    }
}
