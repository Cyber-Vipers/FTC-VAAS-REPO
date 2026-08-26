package org.firstinspires.ftc.teamcode.neuronpathing;

public class MathFunc {
    public static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2.0 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2.0 * Math.PI;
        }
        return radians;
    }

    public static double angleWrapDegrees(double degrees) {
        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }
}
