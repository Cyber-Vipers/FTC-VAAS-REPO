package org.firstinspires.ftc.teamcode.tools;

public class Detection {
    public Detect_Type type;
    public Vector3 flat_position;
    public int april;
    public Detection(Detect_Type type, Vector3 flat_position, int aprils) {
        this.type = type;
        this.flat_position = flat_position;
        this.april = aprils;
    }
}
