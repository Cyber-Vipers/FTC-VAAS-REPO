package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bunsBot.BunsBot;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

@Autonomous(name="BBJ Hanny")
public class CheeksAuto extends LinearOpMode {
    boolean blue = true;
    BunsBot buns =null;

    @Override
    public void runOpMode() throws InterruptedException {

        buns = new BunsBot(hardwareMap, telemetry, this);

//
//        while (opModeInInit()) {
//            if (gamepad1.x) {
//                blue=true;
//            } else if (gamepad1.b) {
//                blue=false;
//            }
//
//            buns.updateOuttake();
//
//            telemetry.addData("Alliance: ", blue ? "Blue" : "Red");
//            telemetry.update();
//        }
//
//        waitForStart();
//        for (int i = 0; i < 50; i++) {
//            buns.outtakeState = BunsBot.States.UP;
//            buns.updateOuttake();
//        }
//
//        sleep(250);
//        buns.backwards();
//        sleep(250);
//        if (blue) {
//            buns.strafeLeft();
//        } else {
//            buns.strafeRight();
//        }
//        sleep(500);
//        buns.drivetrain.setDrivePower(0, 0, 0, 0);

        buns.drivetrain.pinpoint.setPosition(new Pose2D(-49, -49, Math.toRadians(225)));

        buns.outtakeState = BunsBot.States.DOWN;
        buns.resetTimer();
        while (opModeInInit()) {

            if (gamepad1.x) {
                blue = true;
                buns.drivetrain.pinpoint.setPosition(new Pose2D(-49, -49, Math.toRadians(225)));
            } else if (gamepad1.b) {
                blue = false;
                buns.drivetrain.pinpoint.setPosition(new Pose2D(-49, 49, Math.toRadians(128)));
            }
            if (blue) {
                buns.drivetrain.pinpoint.setPosition(new Pose2D(-49, -49, Math.toRadians(225)));
            } else {
                buns.drivetrain.pinpoint.setPosition(new Pose2D(-49, 49, Math.toRadians(128)));
            }
            buns.outtakeState = BunsBot.States.DOWN;
//            buns.drivetrain.pinpoint.setPosition(new Pose2D(-49, 49, Math.toRadians(143)));
            buns.update();
            telemetry.addData("Alliance", (blue ? "Blue" : "Red"));
            telemetry.update();
        }

        waitForStart();

        if (blue) {
            buns.goToPoint(new Pose2D(-47, -47, Math.toRadians(225)),  false, 1);
            shoot();
            buns.intake();
            buns.goToPoint(new Pose2D(-10, -22, Math.toRadians(270)),  true, 1);
            buns.goToPoint(new Pose2D(-10, -51, Math.toRadians(270)),  false, 0.4);
            buns.goToPoint(new Pose2D(-10, -27, Math.toRadians(270)),  true, 0.4);

            buns.goToPoint(new Pose2D(-47, -47, Math.toRadians(225)),  false, 1);
            buns.turnOffIntake();
            buns.sleep(500);
            shoot();
            buns.intake();
            buns.goToPoint(new Pose2D(14, -22, Math.toRadians(270)),  true, 1);

            buns.goToPoint(new Pose2D(14, -51, Math.toRadians(270)),  false, 0.4);
            buns.goToPoint(new Pose2D(14, -27, Math.toRadians(270)),  true, 0.4);

            buns.goToPoint(new Pose2D(-49, -49, Math.toRadians(225)),  false, 1);
            buns.turnOffIntake();
            buns.sleep(500);

            shoot();
            buns.intake();
            buns.goToPoint(new Pose2D(38, -22, Math.toRadians(270)), true, 1);

            buns.goToPoint(new Pose2D(38, -51, Math.toRadians(270)),  false, 0.6);

            buns.goToPoint(new Pose2D(0, -27, Math.toRadians(90)),  true, 1);
            buns.goToPoint(new Pose2D(0, -54, Math.toRadians(90)),  false, 1);
            buns.sleep(1000);
            buns.goToPoint(new Pose2D(0, -27, Math.toRadians(90)),  true, 1);


            buns.goToPoint(new Pose2D(-47, -47, Math.toRadians(233)),  false, 1);
            buns.turnOffIntake();
            buns.sleep(500);
            shoot();

            buns.goToPoint(new Pose2D(-30, -47, Math.toRadians(270)),  false, 1);
        } else {
            buns.goToPoint(new Pose2D(-47, 47, Math.toRadians(128)),  false, 1);

            shoot();
            buns.intake();
            buns.goToPoint(new Pose2D(-14, 23, Math.toRadians(90)),  false, 1);
            buns.goToPoint(new Pose2D(-14, 54, Math.toRadians(90)),  false, 0.6);
            buns.goToPoint(new Pose2D(-4, 45, Math.toRadians(90)),  false, 1);
            buns.turnOffIntake();
            buns.drivetrain.goToPoint(new Pose2D(-4, 60, Math.toRadians(90)),  false, 0.6);
            buns.sleep(650);
            pop();
            buns.goToPoint(new Pose2D(-4, 50, Math.toRadians(90)),  false, 1);


            buns.goToPoint(new Pose2D(-47, 47, Math.toRadians(128)),  false, 1);

            buns.sleep(500);
            shoot();
            buns.intake();
            buns.goToPoint(new Pose2D(12, 23, Math.toRadians(90)),  false, 1);



            buns.goToPoint(new Pose2D(12, 54, Math.toRadians(90)),  false, 0.6);
            buns.goToPoint(new Pose2D(12, 27, Math.toRadians(90)),  false, 0.6);
            pop();


            buns.goToPoint(new Pose2D(-47, 47, Math.toRadians(128)),  false, 1);
            buns.turnOffIntake();

            buns.sleep(400);

            shoot();

            buns.intake();

            buns.goToPoint(new Pose2D(36, 23, Math.toRadians(90)), false, 1);

            buns.goToPoint(new Pose2D(36, 54, Math.toRadians(90)),  false, 0.6);
            buns.goToPoint(new Pose2D(36, 27, Math.toRadians(90)),  false, 0.6);


            buns.goToPoint(new Pose2D(-47, 47, Math.toRadians(128)),  false, 1);
            pop();
            buns.turnOffIntake();

            buns.sleep(400);
            shoot();

            buns.goToPoint(new Pose2D(-30, 47, Math.toRadians(128)),  false, 1);
        }

    }
    public void shoot() {
        buns.outtakeState = BunsBot.States.UP;
        buns.updateOuttake();
        buns.sleep(1000);
        buns.outtakeState = BunsBot.States.DOWN;
        buns.updateOuttake();
        buns.sleep(500);

    }
    public void pop() {
        buns.outtakeState = BunsBot.States.UP;
        buns.updateOuttake();
        buns.sleep(20);
        buns.outtakeState = BunsBot.States.DOWN;

    }
}
