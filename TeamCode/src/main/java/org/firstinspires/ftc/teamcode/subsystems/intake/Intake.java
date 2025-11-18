package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Telemetry telemetry;

    private DcMotorEx intake;

    public enum IntakeState{
        IN,
        OUT,
        OFF
    }

    private IntakeState intakeState = IntakeState.OFF;

    public Intake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intake = hwMap.get(DcMotorEx.class, "intake");
    }

    public void update() {
        switch(intakeState) {
            case IN:
                intake.setPower(1);
                break;
            case OUT:
                intake.setPower(-1);
                break;
            case OFF:
                intake.setPower(0);
                break;
        }
    }

    public void in() {
        intakeState = IntakeState.IN;
    }
    public void off() {
        intakeState = IntakeState.OFF;
    }
    public void out() {
        intakeState = IntakeState.OUT;
    }
}
