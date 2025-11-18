package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2D;


public class Turret {
    private Telemetry telemetry;

    private DcMotorEx turret;
    private PID pid = new PID(0, 0, 0);

    private double targetDegrees = 0;
    private double error = 0;
    private double currentAngle = 0;

    private final double GOALHEADING = 0;
    private final double TURRETRADIUS = 5;

    public Turret(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        turret = hwMap.get(DcMotorEx.class, "turret");
    }

    public void update() {
        pid.update(error);
    }

    private double ticksToDegrees(double ticks) {
        return 0;
    }
    private double degreesToTicks(double degrees) {
        return 0;
    }

    private double angleFromRobotToGoal(double x, double y) {
        return Math.toDegrees(Math.atan((y - Globals.GOALY)/(x - Globals.GOALX)));
    }

    public void adjustFromCamera(double xDegrees) {
        error = xDegrees;
    }

    private void updateTurretTarget(Pose2D pose) {
        double angle = angleFromRobotToGoal(pose.x, pose.y);
        targetDegrees = angle + pose.h;
    }

    public Pose2D poseFromCamera(Pose2D cameraPose) {
        double angleInRadians = Math.toRadians(currentAngle);
        double xOffset = Math.cos(angleInRadians) * TURRETRADIUS;
        double yOffset = Math.sin(angleInRadians) * TURRETRADIUS;

        return new Pose2D(cameraPose.x + xOffset, cameraPose.y + yOffset, cameraPose.h);
    }
}
