package org.firstinspires.ftc.teamcode.neuronpathing;

public class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distance(Point p) {
        return Math.sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y));
    }

    public double slope(Point p) {
        return (y - p.y) / (x - p.x);
    }

    public void updatePoint(double x, double y) {
        this.x = x;
        this.y = y;
    }
}
