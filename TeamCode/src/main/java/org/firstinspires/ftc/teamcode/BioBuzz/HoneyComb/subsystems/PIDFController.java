package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Plain PIDF controller. Not a Subsystem: it has no hardware and nothing to "update".
 * D is taken from the measurement, not the error, so changing the target never causes a
 * derivative kick and you don't need to reset() when the target moves.
 * Call reset() when the mechanism is re-enabled or the encoder is reset.
 */
public class PIDFController {
    private final ElapsedTime timer = new ElapsedTime();

    private double kP, kI, kD, kF;
    private double integralSum = 0;
    private double lastMeasurement = 0;
    private boolean firstRun = true;

    private double maxIntegralOutput = 0.3; // max contribution of the I term to the output
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        setCoefficients(kP, kI, kD, kF);
        reset();
    }

    /** Safe to call every loop: the integral is only cleared when kI actually changes. */
    public void setCoefficients(double kP, double kI, double kD, double kF) {
        if (kI != this.kI) integralSum = 0; // avoid jumps when the I gain changes
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    public void setMaxIntegralOutput(double limit) {
        this.maxIntegralOutput = Math.abs(limit);
    }

    public void reset() {
        integralSum = 0;
        firstRun = true;
        timer.reset();
    }

    /** Uses the stored kF as a constant feedforward. */
    public double calculate(double target, double current) {
        return calculate(target, current, kF);
    }

    /** Lets the caller (e.g. the lift) decide the feedforward. */
    public double calculate(double target, double current, double feedforward) {
        double error = target - current;

        double dt = timer.seconds();
        timer.reset();
        if (dt < 1e-3) dt = 1e-3;

        double p = kP * error;

        // Derivative on measurement: no kick when the target changes.
        // Skipped on the first loop since there's no previous measurement or valid dt yet.
        double d = 0;
        if (!firstRun) {
            d = -kD * (current - lastMeasurement) / dt;
        }
        lastMeasurement = current;

        double unclamped = p + kI * integralSum + d + feedforward;
        boolean saturatedHigh = unclamped >= maxOutput && error > 0;
        boolean saturatedLow = unclamped <= minOutput && error < 0;

        // Integral, clamped by its output contribution so it works in any error units.
        // Don't integrate on the first loop (dt is unknown) or while the output is already maxed
        // out in the same direction (anti-windup).
        if (kI != 0 && !firstRun && !saturatedHigh && !saturatedLow) {
            integralSum += error * dt;
            double maxSum = maxIntegralOutput / Math.abs(kI);
            integralSum = Math.max(-maxSum, Math.min(maxSum, integralSum));
        }
        firstRun = false;

        double output = p + kI * integralSum + d + feedforward;
        return Math.max(minOutput, Math.min(maxOutput, output));
    }
}
