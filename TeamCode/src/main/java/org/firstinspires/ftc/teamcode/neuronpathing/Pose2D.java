package org.firstinspires.ftc.teamcode.neuronpathing;

/**
 * A simple field-centric robot pose: x and y in inches, h (heading) in radians.
 * This is Neuron Pathing's internal pose type, distinct from the SDK's
 * org.firstinspires.ftc.robotcore.external.navigation.Pose2D used by the Pinpoint driver.
 */
public class Pose2D {
    public double x, y, h;

    public Pose2D(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public double distanceToPoint(Pose2D o) {
        return Math.sqrt(Math.pow(o.x - x, 2) + Math.pow(o.y - y, 2));
    }
}
