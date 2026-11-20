package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.LiftShooter;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;

@TeleOp(name = "Honeycomb Robot Teleop", group = "Honeycomb")
@Configurable
public class Teleop extends OpMode {
    LiftShooter liftShooter;
    ElapsedTime timer = new ElapsedTime();
    MecanumDrive mecanumDrive;
    NeuronLocalizer localizer;
    NeuronDrive drive;
    int shootingTime = 3;
    public static int shootingPosition = 935;
    public static int shootingSpeed = 2200;
    @Override
    public void init() {
        liftShooter = new LiftShooter(hardwareMap, telemetry,gamepad1);
        mecanumDrive = new MecanumDrive(hardwareMap,telemetry,gamepad1);
        LiftShooter.shootingPosition = shootingPosition;
        liftShooter.shootSpeed = shootingSpeed;
//        localizer = new NeuronLocalizer(hardwareMap,telemetry,145,35.7);
//        //TODO measure trackwidth in inches https://brainstormz.us/neuron-pathing/docs/installation
//        drive = new NeuronDrive(hardwareMap,localizer,-1);
    }

    @Override
    public void start() {
        liftShooter.intakeLift.moveDown();
    }
//145mm
    @Override
    public void loop() {
        if (shootingPosition != LiftShooterTeleop.shootingPosition) {
            LiftShooter.shootingPosition = shootingPosition;
        }
        if (shootingSpeed != liftShooter.shootSpeed) {
            liftShooter.shootSpeed = shootingSpeed;
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
