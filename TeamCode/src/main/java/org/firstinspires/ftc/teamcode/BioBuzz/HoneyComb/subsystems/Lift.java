package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

public class Lift implements Subsystem {
    public DcMotorEx motorEx;
    private final Telemetry telemetry;
    private int tolerance = 0;
    private int resetPower = -1;
    private int highestPosition = 1200;
    private int lowestPosition = 0;
    private int modifier = 1;
    @Override
    public void update() {
        telemetry.addData("Current Position", motorEx.getCurrentPosition());
        telemetry.addData("Target Position",motorEx.getTargetPosition());
    }

    public Lift(HardwareMap hardwareMap, Telemetry telemetry,int tolerance,int highestPosition, int lowestPosition) {
        motorEx = hardwareMap.get(DcMotorEx.class,"lift");
        this.telemetry = telemetry;
        if (motorEx.getCurrentPosition() <= 100) {
            motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEx.setTargetPositionTolerance(tolerance);
        this.highestPosition = highestPosition;
        this.lowestPosition = lowestPosition;
    }

    public boolean isUp() {
        return plusOrMinus(motorEx.getCurrentPosition(),tolerance,highestPosition);
    }
    public void topOut() {
        setTargetPosition(highestPosition);
    }
    public void configure() {
        resetEncoder();
        motorEx.setTargetPositionTolerance(tolerance);
    }


    public void resetEncoder() {
        motorEx.setTargetPosition(0);
        motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void forceDown() {
        motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorEx.setPower(resetPower);
        if (motorEx.getCurrentPosition() <= lowestPosition) {
            configure();
        }
    }

    public void setTargetPosition(int position) {
        configure();
        motorEx.setTargetPosition(position);
    }

    public void setModifier(int modifier) {
        this.modifier = modifier;
    }

    public void liftUp() {
        int currentPosition = motorEx.getCurrentPosition();
        int modulatedPosition = currentPosition + modifier;
        setTargetPosition(modulatedPosition);
    }

    public void liftDown() {
        int currentPosition = motorEx.getCurrentPosition();
        int modulatedPosition = currentPosition - modifier;
        setTargetPosition(modulatedPosition);
    }

}
