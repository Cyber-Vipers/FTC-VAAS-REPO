package org.firstinspires.ftc.teamcode.Runt;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

@TeleOp(name="RUNT TeleopV1")
public class Base_Teleop extends OpMode {
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    private Follower follower;

    Limelight3A limelight;
    public boolean is_intaking = true;
    DcMotor frontLeft,backLeft,frontRight,backRight,intake;
    OpenCvCamera camera;
    float forward,strafe,turn;
    double denominator;
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        follower = Constants.createFollower(hardwareMap);
        limelight.pipelineSwitch(0);
        /*
         * Starts polling for data.
         */
        limelight.start();
        follower.setMaxPower(0.6);


    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        movement();
        run_intake();
    }


// if x is pressed then stop intake else move constantly
    public void run_intake(){
        if (gamepad2.x) {
            is_intaking = false;
        } else {
            is_intaking = true;
        }
        if (is_intaking) {
           intake.setPower(1);
        } else{
            intake.setPower(0);
        }

    }



    public void movement(){
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        double highestValue;

        double forwardBackwardValue = gamepad1.left_stick_y; //Controls moving forward/backward
        double leftRightValue = gamepad1.left_stick_x * 1.1; //Controls strafing left/right       *the 1.1 multiplier is to counteract any imperfections during the strafing*
        double turningValue = gamepad1.right_stick_x; //Controls turning left/right
        if (gamepad1.left_stick_button) {
            forwardBackwardValue /= 2;
            leftRightValue /= 2;
            turningValue /= 2;
        }
        highestValue = Math.max(Math.abs(forwardBackwardValue) + Math.abs(leftRightValue) + Math.abs(turningValue), 1);

        //Calculates amount of power for each wheel to get the desired outcome
        //E.G. You pressed the left joystick forward and right, and the right joystick right, you strafe diagonally while at the same time turning right, creating a circular strafing motion.
        //E.G. You pressed the left joystick forward, and the right joystick left, you drive like a car and turn left
        if(highestValue >= 0.3){
            frontLeft.setPower((forwardBackwardValue + -leftRightValue + -turningValue) / highestValue);
            backLeft.setPower((forwardBackwardValue - -leftRightValue + -turningValue) / highestValue);
            frontRight.setPower((forwardBackwardValue - -leftRightValue - -turningValue) / highestValue);
            backRight.setPower((forwardBackwardValue + -leftRightValue - -turningValue) / highestValue);
        }
    }

}
