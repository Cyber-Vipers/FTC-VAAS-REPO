package org.firstinspires.ftc.teamcode.Decode.Idontcare;

import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class NorthRobotComposition {
    public static DcMotor frontLeft,frontRight,backRight,backLeft,intake,lift;
    RevRoboticsUltraPlanetaryHdHexMotor flywheel;
    public static CRServo feeder;
    public NorthRobotComposition () {

    }
}
