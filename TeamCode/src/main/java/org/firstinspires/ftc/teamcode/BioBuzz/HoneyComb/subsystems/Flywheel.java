package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

/**
 * Flywheel on the hub's built-in velocity PIDF. Speeds are encoder ticks per second.
 *
 * The hub is only sent a new velocity when the signed target actually changes: unlike
 * setPower(), setVelocity() goes to the hub on every call, and setting it several times a
 * loop to different values made the flywheel briefly chase each of them.
 */
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
    private double sentVelocity = Double.NaN; // last velocity sent to the hub

    @Override
    public void update() {

    }

    /** Sets the speed magnitude (ticks/sec). Direction comes from forward()/reverse(). */
    public void setSpeed(int value) {
        speed = value;
        apply();
    }

    /**
     * Speed AND direction at once (+ = forward, - = reverse), so changing both sends the hub one
     * command instead of a brief wrong one in between.
     */
    public void setTarget(int signedSpeed) {
        reversed = signedSpeed < 0;
        speed = Math.abs(signedSpeed);
        apply();
    }

    /** Signed target velocity in ticks/sec. */
    public double getTargetVelocity() {
        return reversed ? -speed : speed;
    }

    public boolean isUp2Speed() {
        return plusOrMinus(flywheelMotor.getVelocity(), tolerance, getTargetVelocity());
    }

    /** Velocity in ticks/sec (the same unit setSpeed uses). */
    public double getVelocity() {
        return flywheelMotor.getVelocity();
    }

    /** Target speed in ticks/sec, as last given to setSpeed(). Use getVelocity() for the measured speed. */
    public double getSpeed() {
        return speed;
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
        double target = getTargetVelocity();
        if (target != sentVelocity) {
            flywheelMotor.setVelocity(target);
            sentVelocity = target;
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("flywheel (ticks/s)", "%.0f -> %.0f%s", getVelocity(), getTargetVelocity(),
                isUp2Speed() ? "  at speed" : "");
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