package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Lift;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.ToggledMotor;

@TeleOp(name="Honeycomb Lift Shooter", group="Honeycomb")
public class LiftShooter extends OpMode {
    Lift lift;
    Flywheel flywheel;
    ToggledMotor intake;

    // Starting point only. Tune P and F for your flywheel.
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(5.902, 0.295, 0, 14.754);

    public int highestLift = 1062;
    public int lowestLift = 0;
    public int globalTolerance = 20;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap, telemetry, pidfCoefficients, 40);
        intake = new ToggledMotor(hardwareMap, telemetry, "intake");
        lift = new Lift(hardwareMap, telemetry, globalTolerance, highestLift, lowestLift, false);
        lift.resetEncoder();
        // Shooting position
        lift.saveNotablePosition(542);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Lift Position", lift.getPosition());
    }

    @Override
    public void loop() {
        driver1auto();
    }

    public void driver1auto() {
        boolean shooting = gamepad1.right_trigger > 0.5;

        telemetry.addData("Lift Position", lift.getPosition());
        telemetry.addData("Shooting?", shooting);
        telemetry.addData("Flywheel target", flywheel.getTargetVelocity());
        telemetry.addData("Flywheel velocity (ticks/s)", flywheel.getVelocity());
        telemetry.addData("Flywheel mode", flywheel.flywheelMotor.getMode());
        telemetry.addData("Is up to speed", flywheel.isUp2Speed());
        telemetry.addData("Is up?", lift.isUp());
        if (gamepad1.x) {
            lift.liftUp();
        }
        if (gamepad1.y) {
            lift.liftDown();
        }
        if (gamepad1.a) {
            intake.toggle();
        }

        if (gamepad1.bWasPressed()) {
            telemetry.addLine("Toggling direction");
            intake.toggleDirection();
        }
        if (gamepad1.right_bumper) {
            flywheel.setSpeed((int) (flywheel.getSpeed() + 50));
        }
        if (gamepad1.left_bumper) {
            flywheel.setSpeed((int) (flywheel.getSpeed() - 50));
        }
//        if (shooting) {
//            shoot();
//        } else {
//            if (lift.getPosition() >= lift.notablePositions.get(0)) {
//                lift.bottomOut();
//            }
//            flywheel.reverse();
//            flywheel.setSpeed(500);
//            intake.setReverse();
//            intake.On();
//        }
    }

    public void shoot() {

        lift.goToNotablePosition(0);
        if (flywheel.isUp2Speed() && lift.isUp()) {
            intake.setForward();
            intake.On();
        }
        flywheel.forward();
        flywheel.setSpeed(1500);
    }

    public void flower() {
        lift.topOut();
        intake.setReverse();
        intake.On();
        flywheel.forward();
        flywheel.setSpeed(500);
    }
}