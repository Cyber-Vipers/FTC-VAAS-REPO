package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.IntakeConstants;
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
        voltageIgnored = false;
    }
    public boolean voltageIgnored = false;
    public void ignoreVoltage() {
        voltageIgnored = true;
    }

    /**
     * Turns the motor off when it's over the current alert (IntakeConstants.CURRENT_LIMIT_MA, set
     * once in the constructor). Setting the alert and reading getCurrent() every loop were extra
     * hub calls; read getCurrent() only while testing.
     */
    public void voltageControl() {
        if (motorEx.isOverCurrent()) {
            Off();
        }
    }
    public void On() {
//        if (hasCooldown) {
//            if (System.currentTimeMillis() < lastToggle + motorCooldown) {
//                isWaitingCooldown = true;
//            }
//            //Delay until cooldown
//        } else {
            isOn = true;
            motorEx.setPower(1);
//        }
    }

    public void Off() {
//        if (hasCooldown) {
//            if (System.currentTimeMillis() < lastToggle + motorCooldown) {
//                isWaitingCooldown = true;
//            }
//            //Delay until cooldown
//        } else {
            isOn = false;
            motorEx.setPower(0);
//        }
    }

    public void toggle() {
//        if (hasCooldown) {
//            if (System.currentTimeMillis() < lastToggle + motorCooldown) {
//                isWaitingCooldown = true;
//            }
//            //Delay until cooldown
//        } else {
            forceToggle();
//        }

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
        if (isOn) {
            motorEx.setPower(1);
        }

    }
    public void setReverse() {
        motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        if (isOn) {
            motorEx.setPower(1);
        }
    }
    public boolean isOverCurrent() {
        return motorEx.isOverCurrent();
    }
    private void forceToggle() {
        if (isOn) {
            motorEx.setPower(0);
            isOn = false;
        } else {
            motorEx.setPower(1);
            isOn = true;
        }
        lastToggle = System.currentTimeMillis();
    }
    public ToggledMotor(HardwareMap hardwareMap, Telemetry telemetry,String motorName) {
        motorEx = hardwareMap.get(DcMotorEx.class,motorName);
        motorEx.setCurrentAlert(IntakeConstants.CURRENT_LIMIT_MA, CurrentUnit.MILLIAMPS);
        this.motorName = motorName;
        this.telemetry = telemetry;
    }
    public ToggledMotor(HardwareMap hardwareMap, Telemetry telemetry, String motorName, double cooldown, TimeUnit timeUnit) {
        this(hardwareMap, telemetry, motorName);
        hasCooldown = true;
        motorCooldown = timeUnit.toMillis((long) cooldown);
    }

    /** Motor current in mA. A separate hub call each time (not in the bulk read), so use it for testing. */
    public double getCurrentMa() {
        return motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }



}
