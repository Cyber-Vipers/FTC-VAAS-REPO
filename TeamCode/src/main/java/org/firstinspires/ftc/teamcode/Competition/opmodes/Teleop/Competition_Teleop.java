package org.firstinspires.ftc.teamcode.Competition.opmodes.Teleop;/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */



import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher.launchLeft;
import static org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher.launchRight;
import static org.firstinspires.ftc.teamcode.Competition.Subsystems.Sphere_Operator.handle_divert;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.Robot_Composition;
import org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Competition.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Competition.Subsystems.Sphere_Operator;
import org.firstinspires.ftc.teamcode.Runt.opmodes.DecodeRi3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntVisualConstants;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */
@Configurable
@TeleOp(name = "Competition Teleop", group = "Comp" )
//@Disabled
public class Competition_Teleop extends OpMode {
    public static double left_feeder_block = 0.25;
    public static double right_feeder_block = 0;
    public static double right_feeder_up = 0;
    public static double left_feeder_up = 1;
    final double FEED_TIME_SECONDS = 1; //The feeder servos run this long when a shot is requested
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.
    private boolean slowMode = false;
    private boolean correct_me = false;
    private double slowModeMultiplier = 0.5;
    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    public static double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    public static double RIGHT_POSITION = 0;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private Rev2mDistanceSensor colorleft;
    private Rev2mDistanceSensor colorright;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private DcMotor intake = null;
    public DcMotor feeder;
    private Servo leftFeeder = null;
    private Servo rightFeeder = null;
    private Servo diverter = null;
    private Supplier<PathChain> pathChain;
    private Supplier<PathChain> rot;
    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState;
    Robot_Composition robotComposition;
    private LaunchState rightLaunchState;
    public long last_divert_time = 0;
    public long divert_cooldown = 1000;


    private enum DiverterDirection {
        LEFT,
        RIGHT;
    }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState {
        ON,
        OFF;
    }

    private IntakeState intakeState = IntakeState.OFF;

    public static int colorleftgain = 16;
    public static int colorrightgain = 16;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Launcher.LauncherDistance launcherDistance = Launcher.LauncherDistance.CLOSE;
    public Servo limelight_servo;
    public Limelight3A limelight;


    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    public IMU imu;
    double rightBackPower;
    double targetHeading = Math.toRadians(180); // Radians
    PIDFController controller;
    boolean headingLock = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;
        limelight_servo = hardwareMap.get(Servo.class, "limelightServo");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        intake = hardwareMap.get(DcMotor.class, "rollerIntake");
        leftFeeder = hardwareMap.get(Servo.class, "indexLeft");
        rightFeeder = hardwareMap.get(Servo.class, "indexRight");
        diverter = hardwareMap.get(Servo.class, "modulator");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        colorleft = hardwareMap.get(Rev2mDistanceSensor.class, "color2");
        colorright = hardwareMap.get(Rev2mDistanceSensor.class, "color1");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);


        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */


        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        follower = RuntOdometryWheelsConstants.createFollower(hardwareMap);
        controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        robotComposition = new Robot_Composition(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, leftLauncher, rightLauncher, intake, feeder, leftFeeder, rightFeeder,colorleft,colorright,follower,telemetry,diverter,limelight_servo,limelight);

        // if preload set it
        Sphere_Operator.Sphere left_inside = new Sphere_Operator.Sphere(
                new Sphere_Operator.Sphere_Handling_Constants(
                        Sphere_Operator.Ball_Handling_Side.Right,
                        0
                )
        );
        Sphere_Operator.Sphere right_inside = new Sphere_Operator.Sphere(
                new Sphere_Operator.Sphere_Handling_Constants(
                        Sphere_Operator.Ball_Handling_Side.Right,
                        0
                )
        );
        robotComposition.left_inside = left_inside;
        diverter.setDirection(Servo.Direction.REVERSE);
        robotComposition.right_inside = right_inside;
        /*
         * Tell the driver that initialization is complete.
         */
        targetHeading = Math.toRadians(130); // Radians
        PIDFController controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        boolean headingLock = true;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("This is the TELEOP for 20525 for Config 'Silly Robot'"," This Does NOT MOVE (off the table) without input");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("Distance", colorleft.getDistance(DistanceUnit.CM));



    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        leftFeeder.setPosition(left_feeder_block);
        rightFeeder.setPosition(right_feeder_block);
        limelight.setPollRateHz(100);
        limelight.start();
        follower.startTeleopDrive();
    }

    /*
     * Code9 to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    public int last_left_inside = 0;
    public int last_right_inside = 0;
    public long last_dis_change = 0;
    public static long dist_wait = 500;
    public double slow_multiplier = 1.5;

    public boolean overload = false;
    public long overload_time = 0;
    public long overload_wait = 500;
    public int intake_ran = 0;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    public ElapsedTime looptime = new ElapsedTime();
    public boolean can_distance = false;
    public long last_heading_swap = 0;
    public static long heading_cooldown = 0;

    @Override
    public void loop() {
        telemetry.addData("DIVERTER POSITION", diverter.getPosition());
        follower.update();
        looptime.startTime();
        double distance_left = colorleft.getDistance(DistanceUnit.CM);
        double distance_right = colorright.getDistance(DistanceUnit.CM);
        handle_divert(robotComposition,distance_left,distance_right);
//        if (gamepad2.start) {
//            headingLock = false;
//        }
//        if (gamepad2.b && System.currentTimeMillis() >= last_heading_swap + heading_cooldown ) {
//            headingLock = true;
//            targetHeading = Math.toRadians(40);
//            last_heading_swap = System.currentTimeMillis();
//        }
//        if (gamepad2.x && System.currentTimeMillis() >= last_heading_swap + heading_cooldown) {
//            headingLock = true;
//            targetHeading = Math.toRadians(135);
//            last_heading_swap = System.currentTimeMillis();
//        }
//        if (gamepad2.y && System.currentTimeMillis() >= last_heading_swap + heading_cooldown) {
//            headingLock = true;
//            targetHeading = 110;
//            last_heading_swap = System.currentTimeMillis();
//        }
//        if (gamepad2.a) {
//            headingLock = true;
//            targetHeading = Math.toRadians(69);
//            last_heading_swap = System.currentTimeMillis();
//        }


        if (gamepad1.right_bumper) {
            if (last_dis_change +dist_wait <= System.currentTimeMillis()) {
                if (launcherDistance == Launcher.LauncherDistance.FAR) {
                    launcherDistance = Launcher.LauncherDistance.CLOSE;
                    launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                } else {
                    launcherDistance = Launcher.LauncherDistance.FAR;
                    launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                }
                last_dis_change = System.currentTimeMillis();
            }
        }
        mecanumDrive(-gamepad1.left_stick_y - (gamepad1.right_trigger / slow_multiplier), gamepad1.left_stick_x - (gamepad1.right_trigger / slow_multiplier), gamepad1.right_stick_x - (gamepad1.right_trigger / slow_multiplier));
        if (gamepad1.right_trigger >= 0.1) {
            slowMode = true;
        } else {
            slowMode = false;
        }
        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad2.dpad_up) {
            Limelight.look_up(robotComposition);
        }
        if (gamepad2.dpad_down) {
            Limelight.look_down(robotComposition);
        }
        if (gamepad2.dpad_right) {
            Limelight.look_middle(robotComposition);
        }
        if (gamepad1.y) {
            leftLauncher.setVelocity(launcherTarget);
            rightLauncher.setVelocity(launcherTarget);
        } else if (gamepad1.b) { // stop flywheel
            leftLauncher.setVelocity(STOP_SPEED);
            rightLauncher.setVelocity(STOP_SPEED);
        }
        if (gamepad1.xWasPressed()) {
            intakeState = IntakeState.OFF;
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(1);
        }
        if (gamepad1.aWasPressed()){
            switch (intakeState){
                case ON:
                    intakeState = IntakeState.OFF;
                    intake.setPower(0);

                    break;
                case OFF:
                    intakeState = IntakeState.ON;
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(1);
                    break;
            }
        }
        if (gamepad1.left_bumper) {
            if (last_divert_time + divert_cooldown < System.currentTimeMillis()) {
                switch (diverterDirection) {
                    case LEFT:
                        diverterDirection = DiverterDirection.RIGHT;
                        diverter.setPosition(RIGHT_POSITION);
                        break;
                    case RIGHT:
                        diverterDirection = DiverterDirection.LEFT;
                        diverter.setPosition(LEFT_POSITION);
                        break;
                }
                last_divert_time = System.currentTimeMillis();
            }
        }


        /*
         * Now we call our "Launch" function.
         */

        launchLeft(gamepad1.dpadLeftWasPressed(),robotComposition,launcherDistance,0);
        launchRight(gamepad1.dpadRightWasPressed(),robotComposition,launcherDistance,0);
        if (gamepad1.dpad_up) {
            launchRight(true,robotComposition,launcherDistance,0);
            launchLeft(true,robotComposition,launcherDistance,0);
        }


        /*
         * Show the state and motor powers
         */
        // First, tell Limelight which way your robot is facing
        LLResult result = limelight.getLatestResult();

        double robotYaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.RADIANS).firstAngle;
        limelight.updateRobotOrientation(robotYaw);
        telemetry.addData("Is Result Valid", result.isValid());
        for (LLResultTypes.FiducialResult fiducialResult : limelight.getLatestResult().getFiducialResults()) {
            Pose3D poser = fiducialResult.getRobotPoseFieldSpace();
            telemetry.addData("X", poser.getPosition().x);
            telemetry.addData("Y",poser.getPosition().y);
        }
        telemetry.addData("Pipeline",limelight.getLatestResult().getPipelineIndex());
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }
        telemetry.addData("State", leftLaunchState);
        telemetry.addData("launch distance", launcherDistance);
        telemetry.addData("Left Launcher Velocity", leftLauncher.getVelocity());
        telemetry.addData("Right Launcher Velocity", rightLauncher.getVelocity());
        telemetry.addData("Loop Time", looptime.milliseconds());
        looptime.reset();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    public static double turn_multipler = 0.8;

    public double getHeadingError() {


        double headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
        return headingError;
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        controller.setCoefficients(follower.constants.getCoefficientsHeadingPIDF());
        controller.updateError(getHeadingError());
        telemetry.addData("Heading Error", controller.getError());
        if (headingLock) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run() + -gamepad1.right_stick_x);
            telemetry.addData("Locking Heading", controller.getTargetPosition());
        }else{
            telemetry.addData("Not locking heading", -gamepad1.right_stick_x);
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
    }

        //Automated PathFollowing

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.b || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
    }






}