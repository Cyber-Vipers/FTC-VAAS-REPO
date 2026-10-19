package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

import java.util.ArrayList;

public class Lift implements Subsystem {
    public DcMotorEx motorEx;
    private final Telemetry telemetry;
    private int tolerance = 0;
    private int resetPower = -1;
    private int highestPosition = 1200;
    private int lowestPosition = 0;
    public ArrayList<Integer> notablePositions = new ArrayList();
    public double getPosition() { return motorEx.getCurrentPosition();}
    private int modifier = 40;
    @Override
    public void update() {
        telemetry.addData("Current Position", motorEx.getCurrentPosition());
        telemetry.addData("Target Position",motorEx.getTargetPosition());
    }

    public Lift(HardwareMap hardwareMap, Telemetry telemetry,int tolerance,int highestPosition, int lowestPosition,boolean reversed) {
        motorEx = hardwareMap.get(DcMotorEx.class,"lift");
        this.telemetry = telemetry;
        if (motorEx.getCurrentPosition() <= 100) {
            motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (reversed) {
            motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        motorEx.setTargetPosition(0);
        motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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



    public void bottomOut() {
        setTargetPosition(lowestPosition + 20);
    }
    public void configure() {
//        resetEncoder();
        motorEx.setTargetPositionTolerance(tolerance);
    }

    public void saveNotablePosition(Integer position) {
        notablePositions.add(position);
    }
    public void goToNotablePosition(int index) {
        setTargetPosition(notablePositions.get(index));
    }
    public void goToPosition(int position) {
        setTargetPosition(Math.min(position,highestPosition));
    }

    public void overrideGoToPosition(int position) {
        motorEx.setTargetPosition(position);
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
        motorEx.setPower(1);
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
