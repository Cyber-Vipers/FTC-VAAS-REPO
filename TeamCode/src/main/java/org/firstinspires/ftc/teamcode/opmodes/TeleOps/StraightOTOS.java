package org.firstinspires.ftc.teamcode.opmodes.TeleOps;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.OTOS;
import org.firstinspires.ftc.teamcode.subsystems.drive.Pinpoint;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

@TeleOp
public class StraightOTOS extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.update();
            telemetry.update();
        }
    }
}
