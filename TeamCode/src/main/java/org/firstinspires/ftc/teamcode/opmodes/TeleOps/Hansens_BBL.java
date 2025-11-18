package org.firstinspires.ftc.teamcode.opmodes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bunsBot.BunsBot;

@TeleOp(name="BBL Hanny")
public class Hansens_BBL extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BunsBot buns = new BunsBot(hardwareMap, telemetry, this);


        buns.outtakeState = BunsBot.States.DOWN;
        buns.resetTimer();
        while (opModeInInit()) {
            buns.updateOuttake();
        }

        waitForStart();
        buns.reset();

        while (opModeIsActive()) {


            if (gamepad1.right_trigger > 0.5) {
                buns.outtakeState = BunsBot.States.UP;
            } else {
                buns.outtakeState = BunsBot.States.DOWN;
            }

            if (gamepad1.dpad_up) {
                buns.resetOuttake();
            }
            if (gamepad1.right_bumper) {
                buns.footState = BunsBot.States.DOWN;
            } else if (gamepad1.left_bumper) {
                buns.footState = BunsBot.States.UP;
            } else {
                buns.footState = BunsBot.States.HOLD;
            }

            if (gamepad1.x) {
                buns.intake();
            } else if (gamepad1.b) {
                buns.turnOffIntake();
            } else if (gamepad1.y) {
                buns.outtakeOnTheIntake();
            }



            buns.drivetrain.drive(gamepad1);
            buns.updateOuttake();
//            buns.update();
            buns.updateFoot();
            telemetry.addData("Motor Powers: ", buns.outtake1.getPower() + " : " + buns.outtake2.getPower());
            telemetry.addData("Outtake State: ", buns.outtakeState);
            telemetry.addData("Outtake Pos: ", buns.outtake1.getCurrentPosition());
            telemetry.addData("Foot State: ", buns.footState);
            telemetry.update();
        }
    }
}
