package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Lift;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Stick;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.ToggledMotor;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.toggledServo;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;

import java.util.concurrent.TimeUnit;

public class Teleop extends OpMode {
    MecanumDrive mecanumDrive;
    Flywheel leftFlywheel;
    Flywheel rightFlywheel;
    Stick stick;
    Lift lift;
    ToggledMotor liftTransfer,intake;
    toggledServo intakeLift;
    NeuronLocalizer localizer;
    NeuronDrive drive;
    public enum ShootStates {
        Intaking,
        Preparing,
        Shooting
    }

    Servo intakeLiftServo;


    public ShootStates shootState = ShootStates.Intaking;
    public int liftUpPosition = 1200; // REPLACE ME
    public int liftDownPosition = 0;
    public int liftTolerance = 40;
    public int shootTime = 3;

    public double intakeCooldown = 0.2;
    public double intakeLiftUp = 0;
    public double intakeLiftDown = 0.5;
    public int shootingFlywheelSpeed = 1200; // REPLACE ME
    public int intakingFlywheelSpeed = 120;
    ElapsedTime shootTimer = new ElapsedTime();
    public int flyWheelTolerance = 50;
    public static PIDFCoefficients leftPidfCoefficients = new PIDFCoefficients(0,0,0,0);
    public static PIDFCoefficients rightPidfCoefficients = new PIDFCoefficients(0,0,0,0);
    @Override
    public void init() {
//        localizer = new NeuronLocalizer(hardwareMap,telemetry,0,0);
//        drive = new NeuronDrive(hardwareMap,localizer,14);
//        localizer.setPosition(new Pose2D(0,0,0));
        mecanumDrive = new MecanumDrive(hardwareMap,telemetry,gamepad1);
        leftFlywheel = new Flywheel(hardwareMap,telemetry,leftPidfCoefficients,"leftFlywheel",flyWheelTolerance);
        rightFlywheel = new Flywheel(hardwareMap,telemetry,rightPidfCoefficients,"rightFlywheel",flyWheelTolerance);
        stick = new Stick(hardwareMap,telemetry);
        lift = new Lift(hardwareMap,telemetry,liftTolerance,liftUpPosition,liftDownPosition,true,LiftShooter.liftPidf);
        liftTransfer = new ToggledMotor(hardwareMap,telemetry,"liftTransfer",intakeCooldown, TimeUnit.SECONDS);
        intakeLift = new toggledServo(hardwareMap,telemetry,intakeLiftUp,intakeLiftDown,"liftTransfer",true);
        intakeLiftServo = hardwareMap.get(Servo.class,"liftTransfer");
    }

    @Override
    public void start() {
        intakeLift.moveDown();
    }

    @Override
    public void loop() {
//        drive.update();
        controls();
        liftTransfer.update();
        mecanumDrive.update();
        leftFlywheel.update();
        intake.update();

        shoot();
    }

    public void controls() {
        driverOne();
        driverTwo();
        mecanumDrive.update();
    }

    public void driverOne() {
        if (gamepad1.x) {
            //Intake
            shootState = ShootStates.Intaking;
        }
        if (gamepad1.right_trigger > 0.5) {
            //Shoot
            shootState = ShootStates.Preparing;
        }
        if (gamepad1.dpadUpWasPressed()) {
            intakeLiftServo.setPosition(intakeLiftServo.getPosition() + 0.1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            intakeLiftServo.setPosition(intakeLiftServo.getPosition() - 0.1);
        }
        telemetry.addData("Intake Lift Servo Position", intakeLiftServo.getPosition());
    }

    public void driverTwo() {
        if (gamepad2.left_stick_y > 0.5) {
            lift.liftDown();
        }
        if (gamepad2.left_stick_x < -0.5) {
            lift.liftUp();
        }
        if (gamepad2.x && gamepad2.a) {
            lift.resetEncoder();
        }
        if (gamepad2.a && !gamepad2.x) {
            stick.stopPushing();
        }
        if (gamepad2.x && !gamepad2.a) {
            stick.push();
        }
        if (gamepad1.y) {
            if (lift.isUp()) {
                leftFlywheel.setSpeed(shootingFlywheelSpeed);
                rightFlywheel.setSpeed(shootingFlywheelSpeed);
            } else {
                leftFlywheel.setSpeed(intakingFlywheelSpeed);
                leftFlywheel.setSpeed(intakingFlywheelSpeed);
            }
        }

    }

    public void shoot() {
        switch (shootState) {
            case Intaking -> {
                beginIntake();
            }
            case Preparing -> {
                liftTransfer.Off();
                if (lift.isUp() && (leftFlywheel.isUp2Speed() && rightFlywheel.isUp2Speed())) {
                    shootTimer.reset();
                    shootTimer.startTime();
                    shootState = ShootStates.Shooting;
                } else {
                    lift.topOut();
                    leftFlywheel.setSpeed(shootingFlywheelSpeed);
                    leftFlywheel.setSpeed(shootingFlywheelSpeed);
                }
            }
            case Shooting -> {
                if (shootTimer.milliseconds() + shootTime * 1000 >= shootTime * 1000) {
                    shootState = ShootStates.Preparing;
                } else {
                    stick.push();
                }
            }
        }
    }

    public void beginIntake() {
        lift.setTargetPosition(0);
        liftTransfer.On();
        leftFlywheel.reverse();
        rightFlywheel.reverse();
        rightFlywheel.setSpeed(intakingFlywheelSpeed);
        leftFlywheel.setSpeed(intakingFlywheelSpeed);
        stick.stopPushing();
    }
}
