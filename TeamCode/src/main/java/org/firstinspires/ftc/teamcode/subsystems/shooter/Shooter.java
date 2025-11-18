package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private Telemetry telemetry;

    private DcMotorEx flyWheel;

    public enum ShooterState {
        ON,
        OFF,
        CUSTOM
    }
    public ShooterState shooterState = ShooterState.CUSTOM;

    public double speed = 1;

    public Shooter(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        flyWheel = hwMap.get(DcMotorEx.class, "flywheel");
    }

    public void update() {
        switch (shooterState) {
            case ON:
                flyWheel.setPower(speed);
                break;
            case OFF:
                flyWheel.setPower(0);
                break;
            case CUSTOM:
                break;
        }
    }


    public void shoot() { shooterState = ShooterState.ON; }
    public void off() { shooterState = ShooterState.OFF; }



}
