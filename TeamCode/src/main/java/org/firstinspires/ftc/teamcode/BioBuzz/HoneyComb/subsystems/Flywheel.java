package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

@Configurable
public class Flywheel implements Subsystem {

    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public DcMotorEx flywheelMotor;
    public static PIDFCoefficients pidfCoefficients;

    // Not static, so Panels can't overwrite it behind your back.
    private int speed = 0;            // magnitude only, ticks/sec
    private int tolerance;
    private boolean reversed = false; // direction = sign of the velocity

    @Override
    public void update() {

    }

    /** Sets the speed magnitude (ticks/sec). Direction comes from forward()/reverse(). */
    public void setSpeed(int value) {
        speed = value;
        apply();
    }

    /** Signed target velocity in ticks/sec. */
    public double getTargetVelocity() {
        telemetry.addData("Target Velocity", reversed ? -speed : speed);
        return reversed ? -speed : speed;
    }

    public boolean isUp2Speed() {
        return plusOrMinus(flywheelMotor.getVelocity(), tolerance, getTargetVelocity());
    }

    /** Velocity in ticks/sec (the same unit setSpeed uses). */
    public double getVelocity() {
        return flywheelMotor.getVelocity();
    }

    /** Velocity in degrees or radians per second. Don't compare this with setSpeed values. */
    public double getSpeed(AngleUnit angleUnit) {
        return flywheelMotor.getVelocity(angleUnit);
    }

    public boolean isReversed() {
        return reversed;
    }

    // Only touches the motor when the direction actually changes, so it's safe to call every loop.
    public void reverse() {
        if (!reversed) {
            reversed = true;
            apply();
        }
    }

    public void forward() {
        if (reversed) {
            reversed = false;
            apply();
        }
    }

    public void toggleDirection() {
        reversed = !reversed;
        apply();
    }

    private void apply() {
        flywheelMotor.setVelocity(getTargetVelocity());
    }

    public void setPidfCoefficients(PIDFCoefficients pidfCoefficients) {
        Flywheel.pidfCoefficients = pidfCoefficients;
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, PIDFCoefficients pidfCoefficients, int tolerance) {
        this(hardwareMap, telemetry, pidfCoefficients, "flywheel", tolerance);
    }

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, PIDFCoefficients pidfCoefficients, String name, int tolerance) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.tolerance = tolerance;

        flywheelMotor = hardwareMap.get(DcMotorEx.class, name);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setPidfCoefficients(pidfCoefficients);

        telemetry.addLine("Initialized Flywheel!");
        telemetry.update();
    }
}