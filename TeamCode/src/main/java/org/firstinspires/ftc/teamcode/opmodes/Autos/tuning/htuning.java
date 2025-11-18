package org.firstinspires.ftc.teamcode.opmodes.Autos.tuning;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bunsBot.BunsBot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

@Autonomous (name = "H TUNING")
public class htuning extends LinearOpMode {
    private BunsBot buns;

    @Override
    public void runOpMode() throws InterruptedException {
        buns = new BunsBot(hardwareMap, telemetry, this);
//        buns.drivetrain.otos.setPosition(new Pose2D(0, 0,0));
        while (opModeInInit() && !isStopRequested()) {
            buns.update();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            buns.goToPoint(new Pose2D(0,0, Math.toRadians(179)), false, 1);

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
