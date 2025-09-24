package org.firstinspires.ftc.teamcode.Runt.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.function.Supplier;

@TeleOp(name="red 2010")
public class Red extends OpMode {
    public RevBlinkinLedDriver blink;
    public Limelight3A limelight;
    Follower follower;
    DcMotor frontLeft,backLeft,frontRight,backRight;
    public static Pose startingPose;

    private Supplier<PathChain> pathChain;
    float forward,strafe,turn;
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        blink = hardwareMap.get(RevBlinkinLedDriver.class,"blink");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        follower.setStartingPose(new Pose(0,0,0));
        follower.update();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();


    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        follower.update();
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
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
        LLResult result = limelight.getLatestResult();
        if (follower.isBusy()) {
            setting_head = true;
        } else {
            setting_head = false;
        }
        if (gamepad1.a) {
            follower.resumePathFollowing();
            follower.turnToDegrees(120);
        } else {
            follower.pausePathFollowing();

        }

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            for(LLResultTypes.FiducialResult april : result.getFiducialResults()) {
                switch (april.getFiducialId()) {
                    case 21:
                        blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        if(Math.abs(tx) > 9) {
                            follower.resumePathFollowing();
                            follower.turnTo( Math.toRadians(tx) * -1);
                        }
                        telemetry.addData("baller level", tx);
                        break;
                    case 22:
                        blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                        break;
                    case 33:
                        blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
                        break;
                    case 20:
                        blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        break;
                    case 24:
                        blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        break;


//read the code, this ish ow we interface with the limelight
                }
            }

        } else {
            blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            telemetry.addData("Limelight", "No Targets");
        }
        follower.update();
    }

public boolean setting_head = false;
}