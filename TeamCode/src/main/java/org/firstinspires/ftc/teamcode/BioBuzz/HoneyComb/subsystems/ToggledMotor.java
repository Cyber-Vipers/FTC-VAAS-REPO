package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

import java.util.concurrent.TimeUnit;

public class ToggledMotor implements Subsystem {

    public boolean isOn = false;
    public boolean hasCooldown = false;
    private final DcMotorEx motorEx;
    private double motorCooldown = 0;
    private long lastToggle = 0;
    private Telemetry telemetry;
    private final String motorName;

    private boolean isWaitingCooldown = false;

    @Override
    public void update() {
        telemetry.addData(motorName + " is on?", isOn);
        if (isWaitingCooldown) {
            if (System.currentTimeMillis() >= lastToggle + motorCooldown) {
                forceToggle();
                isWaitingCooldown = false;
            }
        }
    }

    public void On() {
        if (hasCooldown) {
            if (System.currentTimeMillis() < lastToggle + motorCooldown) {
                isWaitingCooldown = true;
            }
            //Delay until cooldown
        } else {
            isOn = true;
            motorEx.setPower(1);
        }
    }

    public void Off() {
        if (hasCooldown) {
            if (System.currentTimeMillis() < lastToggle + motorCooldown) {
                isWaitingCooldown = true;
            }
            //Delay until cooldown
        } else {
            isOn = true;
            motorEx.setPower(0);
        }
    }

    public void toggle() {
        if (hasCooldown) {
            if (System.currentTimeMillis() < lastToggle + motorCooldown) {
                isWaitingCooldown = true;
            }
            //Delay until cooldown
        } else {
            forceToggle();
        }

    }
    public void toggleDirection() {
        if (motorEx.getDirection() == DcMotorSimple.Direction.REVERSE) {
            setForward();
        } else {
            setReverse();
        }
    }
    public void setForward() {
        motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        motorEx.setPower(1);
    }
    public void setReverse() {
        motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        motorEx.setPower(1);
    }
    private void forceToggle() {
        if (isOn) {
            motorEx.setPower(0);
        } else {
            motorEx.setPower(1);
        }
        lastToggle = System.currentTimeMillis();
    }
    public ToggledMotor(HardwareMap hardwareMap, Telemetry telemetry,String motorName) {
        motorEx = hardwareMap.get(DcMotorEx.class,motorName);
        this.motorName = motorName;
    }
    public ToggledMotor(HardwareMap hardwareMap, Telemetry telemetry, String motorName, double cooldown, TimeUnit timeUnit) {
        this.motorName = motorName;
        motorEx = hardwareMap.get(DcMotorEx.class,motorName);
        hasCooldown = true;
        motorCooldown = timeUnit.toMillis((long) cooldown);
    }



}
