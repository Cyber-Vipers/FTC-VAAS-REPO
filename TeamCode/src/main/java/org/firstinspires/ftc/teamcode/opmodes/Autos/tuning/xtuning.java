package org.firstinspires.ftc.teamcode.opmodes.Autos.tuning;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bunsBot.BunsBot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

@Autonomous (name = "X TUNING")
public class xtuning extends LinearOpMode {
    private BunsBot buns;

    @Override
    public void runOpMode() throws InterruptedException {
        buns = new BunsBot(hardwareMap, telemetry, this);
        buns.drivetrain.pinpoint.setPosition(new Pose2D(0, 0, Math.toRadians(90)));
        while (opModeInInit() && !isStopRequested()) {
            buns.drivetrain.pinpoint.setPosition(new Pose2D(0, 0, Math.toRadians(90)));

            buns.update();

            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            buns.goToPoint(new Pose2D(24,0, Math.toRadians(90)),  false, 1);
        }

    }

    public void wait(double time) {
        double timeStart = System.currentTimeMillis();
        while (!isStopRequested() && System.currentTimeMillis() - timeStart <= time) {
            buns.update();

            telemetry.update();
        }
    }
}
