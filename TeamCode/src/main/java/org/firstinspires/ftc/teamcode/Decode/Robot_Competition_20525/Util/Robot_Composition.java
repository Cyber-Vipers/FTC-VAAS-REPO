package org.firstinspires.ftc.teamcode.Decode.Robot_Competition_20525.Util;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Decode.Robot_Competition_20525.Subsystems.Sphere_Operator;

public class Robot_Composition {
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public Rev2mDistanceSensor color1;
    public Rev2mDistanceSensor color2;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotorEx leftLauncher = null;
    public DcMotorEx rightLauncher = null;
    public DcMotor intake = null;
    public DcMotor feeder;
    public Servo leftFeeder = null;
    public Servo rightFeeder = null;
    public Servo diverter = null;
    public Telemetry telemetry;
    public Follower follower;
    public Sphere_Operator.Sphere left_inside;
    public Sphere_Operator.Sphere right_inside;
    public Servo limelight_servo;
    public Limelight3A limelight3A;
    
    public Robot_Composition(DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb, DcMotorEx ll, DcMotorEx rl, DcMotor in, DcMotor feed, Servo lf2, Servo rf2, Rev2mDistanceSensor colorone, Rev2mDistanceSensor colortwo, Follower f, Telemetry tele,Servo div,Servo ll_servo, Limelight3A limel) {
        leftFrontDrive = lf;
        rightFrontDrive = rf;
        leftBackDrive = lb;
        rightBackDrive = rb;
        leftLauncher = ll;
        rightLauncher = rl;
        intake = in;
        feeder = feed;
        leftFeeder = lf2;
        rightFeeder = rf2;
        color1 = colorone;
        color2 = colortwo;
        follower = f;
        telemetry = tele;
        diverter = div;
        limelight_servo = ll_servo;
        limelight3A = limel;
    }
}
