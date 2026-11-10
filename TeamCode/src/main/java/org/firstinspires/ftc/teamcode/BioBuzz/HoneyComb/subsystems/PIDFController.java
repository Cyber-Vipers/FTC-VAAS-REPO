package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Plain PIDF controller. Not a Subsystem: it has no hardware and nothing to "update".
 * Call reset() whenever the target changes or the mechanism is re-enabled.
 */
public class PIDFController {
    private final ElapsedTime timer = new ElapsedTime();

    private double kP, kI, kD, kF;
    private double integralSum = 0;
    private double lastError = 0;
    private boolean firstRun = true;

    private double maxIntegralOutput = 0.3; // max contribution of the I term to the output
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        setCoefficients(kP, kI, kD, kF);
        reset();
    }

    public void setCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        integralSum = 0; // avoid jumps when gains change
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
        lastError = 0;
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

        // Integral, clamped by its output contribution so it works in any error units
        if (kI != 0) {
            integralSum += error * dt;
            double maxSum = maxIntegralOutput / Math.abs(kI);
            integralSum = Math.max(-maxSum, Math.min(maxSum, integralSum));
        }
        double i = kI * integralSum;

        // Skip D on the first loop to avoid a derivative kick
        double d = 0;
        if (!firstRun) {
            d = kD * (error - lastError) / dt;
        }
        firstRun = false;
        lastError = error;

        double output = p + i + d + feedforward;
        return Math.max(minOutput, Math.min(maxOutput, output));
    }
}