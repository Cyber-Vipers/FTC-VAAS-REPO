package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

@Configurable
public class Flywheel implements Subsystem {

    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public DcMotorEx flywheelMotor;
    public static PIDFCoefficients pidfCoefficients;
    public static int speed = 0;

    @Override
    public void update() {

    }

    public void setSpeed(int value) {
        speed = value;
        flywheelMotor.setVelocity(speed);
    }

    public void setPidfCoefficients(PIDFCoefficients pidfCoefficients) {
        Flywheel.pidfCoefficients = pidfCoefficients;
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
    }


    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry,PIDFCoefficients pidfCoefficients) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,pidfCoefficients);
        telemetry.addLine("Initialized Flywheel!");
        telemetry.update();
    }
    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, PIDFCoefficients pidfCoefficients, String name) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class,name);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,pidfCoefficients);
        telemetry.addLine("Initialized Flywheel!");
        telemetry.update();
    }
}
