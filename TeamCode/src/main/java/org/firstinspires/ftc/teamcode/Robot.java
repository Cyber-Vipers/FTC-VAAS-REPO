package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hwMap;

    public Drivetrain drivetrain;
    public Limelight limelight;
    private LinearOpMode opMode;

    public Turret turret;

    public Robot(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hwMap = hwMap;
        drivetrain = new Drivetrain(hwMap, telemetry);
//        limelight = new Limelight(hwMap, telemetry);
    }

    public void update() {
        limelight.update();
    }

    public void goToPoint(Pose2D pose, boolean through) {
        drivetrain.goToPoint(pose, through, 1);
        while (!opMode.isStopRequested() && drivetrain.isBusy() ) {
            update();
        }
    }

    public void limelightInfo() {
        turret.adjustFromCamera(limelight.getTargetX());
    }

    public Pose2D getPos() {
        return drivetrain.poseFromCamera(turret.poseFromCamera(limelight.getRobotPos()));
    }
}
