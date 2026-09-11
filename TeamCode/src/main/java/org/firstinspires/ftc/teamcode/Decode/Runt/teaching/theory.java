package org.firstinspires.ftc.teamcode.Decode.Runt.teaching;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;

import java.util.function.Supplier;
@Disabled
public class theory extends OpMode {
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    public Follower follower;
    public static Pose startingPose;
    public DcMotor outtake,backLeft,backRight;
    private boolean automatedDrive;
    private double slowModeMultiplier = 0.5;
    private boolean slowMode = false;
    public Limelight3A limelight;
    private Supplier<PathChain> pathChain;
    private Supplier<PathChain> rot;

    @Override
    public void init() {
        follower = RuntOdometryWheelsConstants.createFollower(hardwareMap);
        outtake = hardwareMap.get(DcMotor.class,"outtake");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        follower.update();
        if (gamepad1.left_stick_y >= 0.1 && gamepad1.left_stick_y != 0) {
            backLeft.setPower(1);
            backRight.setPower(1);
        } else{
            backRight.setPower(-1);
            backLeft.setPower(-1);
        }
        if (gamepad1.left_stick_x >= 0.1 && gamepad1.left_stick_x != 0) {
            backLeft.setPower(1);
            backRight.setPower(-1);
        } else {
            backRight.setPower(-1);
            backLeft.setPower(1);
        }
        



    }
}
