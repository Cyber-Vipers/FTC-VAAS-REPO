package org.firstinspires.ftc.teamcode.neuronpathing;

/**
 * A basic PID controller used by NeuronDrive to hold/approach a target point
 * once the robot is off the spline (DriveState.PID / CLOSEPID / HOLD_POS).
 */
public class PID {
    public double p;
    public double i;
    public double d;

    public PID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    double integral = 0;
    long lastLoopTime = System.nanoTime();
    double lastError = 0;
    double loopTime = 0.0;
    int counter = 0;

    public void resetIntegral() {
        integral = 0;
    }

    public double update(double error) {
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10000000;
        }
        if (Math.signum(lastError) != Math.signum(error)) {
            resetIntegral();
        }
        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime) / 1000000000.0;
        lastLoopTime = currentTime;

        double proportion = p * error;
        integral += error * loopTime;
        double derivative = d * (error - lastError) / loopTime;

        lastError = error;
        counter++;

        return proportion + integral * i + derivative;
    }

    public void updatePID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}
