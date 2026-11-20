package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

public class LiftShooter implements Subsystem {
    public final PIDControlledLift lift;
    public final Flywheel flywheel;
    public final ToggledMotor intake, liftTransfer;
    public final toggledServo intakeLift;
    private final Telemetry telemetry;
    public boolean hasShot = false;
    public boolean hasQueued = false;
    public boolean shooting = false;
    public int highestLift = 1762;
    public int lowestLift = 0;
    public int shootSpeed = 2300;
    public static int intakeSpeed = 1000;
    public static double intakeLiftUp = -0.45;
    public static boolean intakeOn = false;
    public static int shootingPosition = 1500;
    public static double intakeLiftDown = 0.3;
    public static boolean autoMode = true;
    public static int globalTolerance = 100;
    private final PIDFCoefficients liftPidf = new PIDFCoefficients(0.0028,0,0.0003,0.039);
    private final PIDFCoefficients flywheelPIDF = new PIDFCoefficients(16.892, 0, 0.507, 12.701);
    @Override
    public void update() {
        if (lift.notablePositions.get(0) != shootingPosition) {
            lift.notablePositions.set(0,shootingPosition);
        }
        if (flywheel.getSpeed() != shootSpeed) {
            flywheel.setSpeed(shootSpeed);
        }
        telemetry.addData("Lift Position", lift.getPosition());
        telemetry.addData("Flywheel Target", flywheel.getTargetVelocity());
        telemetry.addData("Flywheel Velocity (ticks/s)", flywheel.getVelocity());
        telemetry.addData("Flywheel Mode", flywheel.flywheelMotor.getMode());
        telemetry.addData("Is up to speed", flywheel.isUp2Speed());
        telemetry.addData("Is up?", lift.isUp());
        lift.update();
        flywheel.update();
        liftTransfer.update();
        intake.update();
        intakeLift.update();
        liftTransfer.voltageControl();
        if (shooting) {
            makeShot();
        } else {
            liftTransfer.setReverse();
//            liftTransfer.On();
            if (hasShot) {
                liftTransfer.On();
            }
            hasShot = false;
            hasQueued = false;
            if (lift.plusOrMinus(lift.getPosition(),100,lift.notablePositions.get(0))) {
                lift.bottomOut();
            }

            flywheel.reverse();
            flywheel.setSpeed(intakeSpeed);

            if (lift.isDown() && flywheel.isUp2Speed()) {
                intakeLift.moveDown();
            }
        }
        if (!intakeOn) {
            intake.Off();
        }
        intakeOn = false;
    }

    public void outtake() {
        liftTransfer.setForward();
        intake.setForward();
        intake.On();
        flywheel.forward();
        intakeOn = true;
    }


    public void intake() {
        liftTransfer.setReverse();
        intake.setReverse();
        intake.On();
        intakeOn = true;
    }

    public void turnOff() {
        intake.Off();
    }


    private void makeShot() {
        if (!hasQueued) {
            hasQueued = true;
            lift.goToNotablePosition(0);
        }
//        liftTransfer.Off(); // TESTING ONLY
//        liftTransfer.ignoreVoltage();

        boolean shooting = flywheel.isUp2Speed() && lift.plusOrMinus(lift.getPosition(),globalTolerance, lift.notablePositions.get(0));
        if (!hasShot && !shooting) {
//            liftTransfer.On();
            liftTransfer.setForward();
        }
        telemetry.addData("Can shoot?: ",shooting);
        if (shooting) {
            hasShot = true;
            liftTransfer.setForward();
            liftTransfer.On();
        } else {
            if (!hasShot && !shooting) {
//                telemetry.addLine("Intaking on Transfer");
//                liftTransfer.setReverse();
//                liftTransfer.On();
            }
        }
        intake.On();
        intake.setReverse();
        flywheel.setSpeed(0);
        flywheel.forward();
        flywheel.setSpeed(shootSpeed);
    }



    public LiftShooter(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        flywheel = new Flywheel(hardwareMap, telemetry,flywheelPIDF,globalTolerance + 30);
        intake = new ToggledMotor(hardwareMap, telemetry, "intake");
        lift = new PIDControlledLift(hardwareMap, telemetry, globalTolerance, highestLift, lowestLift, false,liftPidf);
        lift.resetEncoder();
        liftTransfer = new ToggledMotor(hardwareMap,telemetry,"transfer");
        intakeLift = new toggledServo(hardwareMap,telemetry,intakeLiftUp,intakeLiftDown,"liftTransfer",true,true);
        lift.saveNotablePosition(shootingPosition);
    }
}
