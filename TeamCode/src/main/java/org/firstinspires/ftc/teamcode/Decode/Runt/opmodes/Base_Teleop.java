package org.firstinspires.ftc.teamcode.Decode.Runt.opmodes;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;

import java.util.function.Supplier;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Configurable

public class Base_Teleop extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    public Limelight3A limelight;
    public DcMotor Intake;
    public DcMotorEx outtake;

    private Supplier<PathChain> pathChain;
    private Supplier<PathChain> rot;

    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private boolean correct_me = false;
    private double slowModeMultiplier = 0.5;
    private boolean is_running = false;
    long last_intake_flip = 0;
    long last_outtake_flip = 0;
    long last_out_take_position = 0;
    long rpm = 0;

    @Override
    public void init() {
        
        follower = RuntOdometryWheelsConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        rot = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose,follower::getPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,Math.toRadians(120),0.8))
                .build();
        telemetry.addData("This is a base teleop for the RUNT for path following", "This DOES run (off the table)");
    }

    @Override
    public void start() {

        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        limelight.setPollRateHz(100);
        limelight.start();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        rpm = outtake.getCurrentPosition() - last_out_take_position;
        last_out_take_position = outtake.getCurrentPosition();


        LLResult result = limelight.getLatestResult();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.a) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        if (gamepad1.x) {
            telemetry.addData("Heading ", Math.toDegrees(follower.getHeading()));
                follower.turnTo(Math.toRadians(120));

            telemetry.addLine("triggered");
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.b || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.right_bumper) {
            slowMode = !slowMode;
        }


        if (gamepad1.x && last_outtake_flip + 500 <= System.currentTimeMillis()){

        }
        
        if (gamepad1.y && last_intake_flip + 500 <= System.currentTimeMillis()) {
            if (is_running == true) {
                is_running = false;
            } else {
                is_running = true;
            }
            last_intake_flip = System.currentTimeMillis();
        }

        if (is_running) {
            Intake.setPower(1);
        } else {
            Intake.setPower(0);
        }
        //Optional way to change slow mode strength
        if (gamepad1.x) {
            slowModeMultiplier += 0.25;
        }




    }
}