package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Lift;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.PIDControlledLift;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.ToggledMotor;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.toggledServo;

@Configurable
@TeleOp(name="Honeycomb Lift Shooter", group="Honeycomb")
public class LiftShooter extends OpMode {
    PIDControlledLift lift;
    Flywheel flywheel;
    ToggledMotor intake, liftTransfer;
    toggledServo intakeLift;
    Servo intakeLiftServo;

    // Starting point only. Tune P and F for your flywheel.
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(5.902, 0.295, 0, 14.754);

    public int highestLift = 1062;
    public int lowestLift = 0;
    public int shootSpeed = 1500;
    public static int intakeSpeed = 1000;
    public static double intakeLiftUp = -0.45;

    public static int shootingPosition = 540;
    public static double intakeLiftDown = 0.35;
    public static boolean autoMode = true;
    public static int globalTolerance = 50;
    public static PIDFCoefficients liftPidf = new PIDFCoefficients(1,1,1,1);

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap, telemetry, pidfCoefficients, 40);
        intake = new ToggledMotor(hardwareMap, telemetry, "intake");
        lift = new PIDControlledLift(hardwareMap, telemetry, globalTolerance, highestLift, lowestLift, false,liftPidf);
        lift.resetEncoder();
        liftTransfer = new ToggledMotor(hardwareMap,telemetry,"transfer");
        // Shooting position
        lift.saveNotablePosition(shootingPosition);
        intakeLift = new toggledServo(hardwareMap,telemetry,intakeLiftUp,intakeLiftDown,"liftTransfer",true);
        intakeLiftServo = hardwareMap.get(Servo.class,"liftTransfer");
        intakeLiftServo.setPosition(0);
        intakeLiftServo.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void init_loop() {
        telemetry.addData("Lift Position", lift.getPosition());
        telemetry.addData("Flywheel Target", flywheel.getTargetVelocity());
        telemetry.addData("Flywheel Velocity (ticks/s)", flywheel.getVelocity());
        telemetry.addData("Flywheel Mode", flywheel.flywheelMotor.getMode());
        telemetry.addData("Is up to speed", flywheel.isUp2Speed());
        telemetry.addData("Is up?", lift.isUp());
        intakeLift.moveDown();
    }

    @Override
    public void start() {
        intakeLift.moveDown();
        intake.toggleDirection();
        liftTransfer.toggleDirection();
        flywheel.toggleDirection();
        flywheel.setSpeed(1000);
        liftTransfer.On();
    }
    @Override
    public void loop() {
        lift.update();
        flywheel.update();
        liftTransfer.update();
        intake.update();
        intakeLift.update();
        liftTransfer.voltageControl();
        driver1();
        if (lift.pidf() != liftPidf) {
            lift.setPidf(liftPidf);
        }
    }

    public void driver1() {
        boolean shooting = gamepad1.right_trigger > 0.5;
        boolean outtaking = gamepad1.left_trigger > 0.5;
        telemetry.addData("Lift Position", lift.getPosition());
        telemetry.addData("Shooting?", shooting);
        telemetry.addData("Flywheel target", flywheel.getTargetVelocity());
        telemetry.addData("Flywheel velocity (ticks/s)", flywheel.getVelocity());
        telemetry.addData("Flywheel Mode", flywheel.flywheelMotor.getMode());
        telemetry.addData("Is up to speed", flywheel.isUp2Speed());
        telemetry.addData("Is up?", lift.isUp());
        telemetry.addData("Automode is active?: ", autoMode);
        if (gamepad1.startWasPressed() && gamepad1.shareWasPressed()) {
            autoMode = !autoMode;
        }
        if (gamepad1.dpadUpWasPressed()) {
            intakeLiftServo.setPosition(intakeLiftServo.getPosition() + 0.1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            intakeLiftServo.setPosition(intakeLiftServo.getPosition() - 0.1);
        }
        if (gamepad2.b) {
            flywheel.setSpeed(1000);
        }
        if (gamepad2.a) {
            flywheel.setSpeed(1500);
        }
        if (gamepad2.x) {
            flywheel.setSpeed(0);
            flywheel.toggleDirection();
        }
        if (!autoMode) {
            if (gamepad2.yWasPressed()) {
                flywheel.toggleDirection();
            }
            if (gamepad1.x) {
                lift.liftUp();
            }
            if (gamepad1.y) {
                lift.liftDown();
            }
            if (gamepad1.right_trigger > 0.5) {
                if (intakeLift.isUp()) {
                    intakeLift.moveDown();
                } else {
                    intakeLift.moveUp();
                }
            }
            if (gamepad1.a) {
                intake.toggle();
                liftTransfer.toggle();
            }


            if (gamepad1.bWasPressed()) {
                telemetry.addLine("Toggling direction");
                intake.toggleDirection();
                liftTransfer.toggleDirection();
            }
            if (gamepad1.right_bumper) {
                flywheel.setSpeed((int) (flywheel.getSpeed() + 50));
            }
            if (gamepad1.left_bumper) {
                flywheel.setSpeed((int) (flywheel.getSpeed() - 50));
            }
        } else {
            if (gamepad1.a) {
                intake.setForward();
                intake.On();
            }
            if (gamepad1.b) {
                intake.setReverse();
                intake.On();
            }
            if (!gamepad1.a && !gamepad1.b) {
                intake.Off();
            }
            if (outtaking) {
                flywheel.forward();
                intake.setReverse();
                return;
            }
            if (gamepad1.start) {
                intake.setForward();
                intake.On();
            }
            if (gamepad1.left_bumper) {
                intakeLift.nudgeDown();
            }
            if (shooting) {
                shoot();
            } else {
                liftTransfer.setReverse();
                if (hasShot) {
                    liftTransfer.On();
                }
                hasShot = false;
                if (lift.plusOrMinus(lift.getPosition(),100,lift.notablePositions.get(0))) {
                    lift.bottomOut();
                }

                flywheel.reverse();
                flywheel.setSpeed(intakeSpeed);

                if (lift.isDown() && flywheel.isUp2Speed()) {
                    intakeLift.moveDown();
                }

            }
        }
    }
    public boolean hasShot = false;
    public void shoot() {
        lift.goToNotablePosition(0);
        liftTransfer.Off(); // TESTING ONLY
//        liftTransfer.ignoreVoltage();
        telemetry.addData("Can shoot?: ",flywheel.isUp2Speed() && lift.plusOrMinus(lift.getPosition(),globalTolerance,lift.notablePositions.get(0)));
        if (flywheel.isUp2Speed() && lift.plusOrMinus(lift.getPosition(),globalTolerance,lift.notablePositions.get(0))) {
            hasShot = true;
//            liftTransfer.setForward();
//            liftTransfer.On();
        } else {
            if (hasShot == false) {
//                liftTransfer.setReverse();
//                liftTransfer.On();
            }
        }
        intake.Off();
        intake.setForward();
        intakeLift.nudgeDown();
        flywheel.setSpeed(0);
//        flywheel.forward();
//        flywheel.setSpeed(shootSpeed);
    }

    public void flower() {
        lift.topOut();
        intake.setReverse();
        intake.On();
        flywheel.forward();
        flywheel.setSpeed(intakeSpeed);
    }
}
