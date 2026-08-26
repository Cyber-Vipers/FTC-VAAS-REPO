package org.firstinspires.ftc.teamcode.neuronpathing;

/**
 * A 2D line, optionally bounded to an X range. Kept around as a small geometry
 * helper for teams building their own field-zone or boundary logic on top of
 * Neuron Pathing; the core path follower does not depend on it.
 */
public class Line {
    double slope;
    Point point;
    double lowX = Integer.MIN_VALUE;
    double highX = Integer.MAX_VALUE;

    public Line(double slope) {
        this.slope = slope;
        this.point = new Point(0, 0);
    }

    public Line(double slope, double lowX, double highX) {
        this.slope = slope;
        this.point = new Point(0, 0);
        this.lowX = lowX;
        this.highX = highX;
    }

    public Line(double slope, Point point) {
        this.slope = slope;
        this.point = point;
    }

    public Line(double slope, Point point, double lowX, double highX) {
        this.slope = slope;
        this.point = point;
        this.lowX = lowX;
        this.highX = highX;
    }

    public void setLine(double slope, Point point, double lowX, double highX) {
        this.slope = slope;
        this.point = point;
        this.lowX = lowX;
        this.highX = highX;
    }

    public Point pointAt(double x) {
        double y = point.y + slope * (x - point.x);
        return new Point(x, y);
    }

    public double yAt(double x) {
        return point.y + slope * (x - point.x);
    }

    public boolean intersectWithinRange(Line o, double lowX, double highX) {
        double x1 = point.x;
        double y1 = point.y;

        double slope2 = o.slope;
        double x2 = o.point.x;
        double y2 = o.point.y;

        double x = (y2 - y1 - slope2 * x2 + slope * x1) / (slope - slope2);

        return (x >= Math.max(lowX, o.lowX) && x <= Math.min(highX, o.highX));
    }
}
