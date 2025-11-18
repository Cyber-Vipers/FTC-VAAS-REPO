package org.firstinspires.ftc.teamcode.utils;

public class Line {
    double slope;
    Point point;
    double lowX, highX;

    public Line(double slope) {
        this.slope = slope;
        this.point = new Point(0, 0f);
    }
    public Line(double slope, double lowX, double highX) {
        this.slope = slope;
        this.point = new Point(0, 0f);
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

    public Point pointAt(double x ) {
        double y = point.y + slope*(x - point.x);
        return new Point(x, y);
    }
    public double yAt(double x ) {
        double y = point.y + slope*(x - point.x);
        return y;
    }

    public boolean intersectWithinRange(Line o, double lowX, double highX) {
        double x1 = point.x;
        double y1 = point.y;
        double slope2 = o.slope;
        double x2 = o.point.x;
        double y2 = o.point.y;

        double x = (slope*x1 - y1 - slope*x2 + y2)/(slope-slope2);

        return (x >= Math.max(lowX, this.lowX)&& x <= Math.min(highX, this.highX));

    }
}
