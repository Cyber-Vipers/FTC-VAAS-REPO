package org.firstinspires.ftc.teamcode.neuronpathing;

import static org.firstinspires.ftc.teamcode.neuronpathing.MathFunc.angleWrap;

import java.util.ArrayList;
import java.util.List;

/**
 * Builds a dense list of Pose2D points along a cubic Hermite spline between
 * waypoints, which NeuronDrive then pure-pursuits along. This is the same
 * curve math the Neuron Path Planner tool replicates in JS for its live preview,
 * so a path drawn in the planner matches what the robot will actually follow.
 */
public class Spline {
    public List<Pose2D> points = new ArrayList<>();

    /**
     * Add a waypoint to the spline.
     * @param pose the target pose to spline to
     * @param reversed whether the robot should travel this segment backwards
     * @param factor controls how "aggressively" the curve pulls toward each heading
     *               tangent; 1.0 is a reasonable default, raise it for wider curves
     */
    public void addPoint(Pose2D pose, boolean reversed, double factor) {
        if (points.isEmpty()) {
            points.add(pose);
            return;
        }

        Pose2D start = points.get(points.size() - 1);

        double x1 = start.x, y1 = start.y, h1 = start.h + (reversed ? Math.PI : 0);
        double x2 = pose.x, y2 = pose.y, h2 = pose.h + (reversed ? Math.PI : 0);

        double v = factor * Math.hypot(x2 - x1, y2 - y1);

        double dx = x1;
        double cx = v * Math.cos(h1);
        double bx = 3 * x2 - v * Math.cos(h2) - 2 * cx - 3 * dx;
        double ax = x2 - bx - cx - dx;

        double dy = y1;
        double cy = v * Math.sin(h1);
        double by = 3 * y2 - v * Math.sin(h2) - 2 * cy - 3 * dy;
        double ay = y2 - by - cy - dy;

        Pose2D prev = start;

        for (double t = 0; t <= 1.0; t += 0.001) {
            double x = dx + cx * t + bx * t * t + ax * t * t * t;
            double y = dy + cy * t + by * t * t + ay * t * t * t;

            double dxdt = cx + 2 * bx * t + 3 * ax * t * t;
            double dydt = cy + 2 * by * t + 3 * ay * t * t;
            double heading = angleWrap(Math.atan2(dydt, dxdt) + (reversed ? Math.PI : 0));

            Pose2D point = new Pose2D(x, y, heading);

            if (point.distanceToPoint(prev) > 2.0) {
                points.add(point);
                prev = point;
            }
        }

        Pose2D end = new Pose2D(x2, y2, pose.h);
        if (end.distanceToPoint(points.get(points.size() - 1)) > 1e-6) {
            points.add(end);
        } else {
            points.set(points.size() - 1, end);
        }
    }
}
