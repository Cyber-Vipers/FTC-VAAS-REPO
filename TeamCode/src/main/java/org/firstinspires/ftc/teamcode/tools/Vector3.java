package org.firstinspires.ftc.teamcode.tools;

public class Vector3 {
    private double x = 0;
    private double y = 0;
    private double z = 0;
    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getZ() {
        return z;
    }
}