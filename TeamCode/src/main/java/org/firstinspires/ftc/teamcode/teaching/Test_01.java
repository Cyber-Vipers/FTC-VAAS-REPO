package org.firstinspires.ftc.teamcode.teaching;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Competition.opmodes.Teleop.Competition_Teleop;
import org.firstinspires.ftc.teamcode.Competition.sensors.limelight_thinker;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;

public class Test_01 extends OpMode {
    private static final Object LAUNCHER_CLOSE_TARGET_VELOCITY = 1000;
    private static final Object LAUNCHER_CLOSE_MIN_VELOCITY = 1300;
    private static final Object LAUNCHER_FAR_TARGET_VELOCITY = 1450;

    private static final Object LAUNCHER_FAR_MIN_VELOCITY = 1400;

    public Follower follower;
    private DcMotor LeftFrontDrive;
    private DcMotor RightFrontDrive;
    private DcMotor LeftRearDrive;
    private DcMotor RightRearDrive;
    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;
    private DcMotor intake;
    private DcMotor leftFeeder;
    private DcMotor rightFeeder;
    private DcMotor diverter;
    private DcMotor outtake;
    

    @Override
    public void init() {
        follower = RuntOdometryWheelsConstants.createFollower(hardwareMap);
        LeftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        RightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        LeftRearDrive = hardwareMap.get(DcMotor.class, "LeftRearDrive");
        RightRearDrive = hardwareMap.get(DcMotor.class, "RightRearDrive");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(DcMotor.class, "leftFeeder");
        rightFeeder = hardwareMap.get(DcMotor.class, "rightFeeder");
        diverter = hardwareMap.get(DcMotor.class, "diverter");
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        

        //connects pedropathing to the robot hardware
    }
    public void loop() {
        telemetry.addData("Status", "Running");
        follower.update();
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

       
        
        
    }

    public void launchLeft() {
        leftLauncher.setVelocity((Double) LAUNCHER_CLOSE_TARGET_VELOCITY);
    }



    void mecanumDrive(double forward, double strafe, double rotate){

        boolean automatedDrive = false;
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            boolean slowMode = false;
            float slowModeMultiplier = 0;
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * 0.8,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier * 0.8,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.b || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.right_bumper) {
            Competition_Teleop.LauncherDistance launcherDistance = Competition_Teleop.LauncherDistance.FAR;
            Object launcherMin;
            Object launcherTarget;
            if (launcherDistance == Competition_Teleop.LauncherDistance.FAR) {
                launcherDistance = Competition_Teleop.LauncherDistance.CLOSE;
                launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
            } else {
                launcherDistance = Competition_Teleop.LauncherDistance.CLOSE;
                launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                launcherMin = LAUNCHER_FAR_MIN_VELOCITY;


            }
        }
    }
}
