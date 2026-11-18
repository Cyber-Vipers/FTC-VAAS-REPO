package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.LiftShooter;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.MecanumDrive;

@TeleOp(name = "Honeycomb Robot Teleop", group = "Honeycomb")
public class Teleop extends OpMode {
    LiftShooter liftShooter;
    ElapsedTime timer = new ElapsedTime();
    MecanumDrive mecanumDrive;
    int shootingTime = 3;
    public static int shootingPosition = 900;
    int shootingSpeed = 2200;
    @Override
    public void init() {
        liftShooter = new LiftShooter(hardwareMap, telemetry,gamepad1);
        mecanumDrive = new MecanumDrive(hardwareMap,telemetry,gamepad1);
        LiftShooter.shootingPosition = shootingPosition;
        liftShooter.shootSpeed = shootingSpeed;
    }

    @Override
    public void start() {
        liftShooter.intakeLift.moveDown();
    }

    @Override
    public void loop() {
        if (shootingPosition != org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.teleop.LiftShooter.shootingPosition) {
            LiftShooter.shootingPosition = shootingPosition;
        }
        driverOne();
        mecanumDrive.update();
        liftShooter.update();
        telemetry.update();
    }

    public void driverOne() {
        /*
            A = Outtake
            B = Intake
            Right Trigger = Shoot
            Thumbsticks = move
         */
        if (gamepad1.y) {
            liftShooter.outtake();
        }
        if (gamepad1.left_trigger >= 0.2) {
            liftShooter.liftTransfer.On();
            liftShooter.intake();
            liftShooter.lift.bottomOut();
        } else {
            if (!liftShooter.shooting) {
                liftShooter.liftTransfer.Off();
            }

        }
        if (timer.milliseconds() >= shootingTime * 1000) {
            liftShooter.shooting = false;
        }
        if (!liftShooter.shooting && gamepad1.right_trigger >= 0.2) {
            timer.reset();
            timer.startTime();
            liftShooter.shooting = true;
        }


    }
}
