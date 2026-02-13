package org.firstinspires.ftc.teamcode.Robot_Competition_20525.opmodes.Teleop;/*   MIT License
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

import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher.launchLeft;
import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher.launchRight;
import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Sphere_Operator.handle_divert;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Util.Robot_Composition;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Sphere_Operator;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;

import java.util.function.Supplier;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */
@Configurable
@TeleOp(name = "Competition Teleop", group = "Comp" )
//@Disabled
public class Competition_Teleop extends OpMode {
    public static PIDFCoefficients flywheel_tuning = new PIDFCoefficients(500, 0, 0, 2.5);
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.
    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    public static double LEFT_POSITION = 0.5962; //the left and right position for the diverter servo
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
    public static boolean limelight_detection_enabled = false;


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
    private Follower follower;
    public static Pose startingPose = new Pose(63.5,8,Math.toRadians(90)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Launcher.LauncherDistance launcherDistance = Launcher.LauncherDistance.CLOSE;
    public Servo limelight_servo;
    public Limelight3A limelight;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    // Setup a variable for each drive wheel to save power level for telemetry
    public boolean manual_override_diverter = false;
    public IMU imu;
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
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        targetHeading = Math.toRadians(130); // Radians
        PIDFController controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        boolean headingLock = true;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("This is the TELEOP for 20525 for Config 'Silly Robot'"," This Does NOT MOVE (off the table) without input");
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
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
        Launcher.shut_gates(robotComposition);
        Launcher.spinup(robotComposition,launcherDistance,0);
        Limelight.look_up(robotComposition);
        //)
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
    public int spam_mult = 1;
    public boolean is_blue = false;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    public static boolean is_configuring = false;



    @Override
    public void loop() {
        looptime.startTime();
        follower.update();
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,flywheel_tuning);
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,flywheel_tuning);
        double distance_left = colorleft.getDistance(DistanceUnit.CM);
        double distance_right = colorright.getDistance(DistanceUnit.CM);
        panelsField.moveCursor(follower.getPose().getX(), follower.getPose().getY());
        if (rightLauncher.getVelocity() >= Launcher.launcherTarget) {
            gamepad1.setLedColor(0,1,0,100);
        }
        if (gamepad2.a) {
            manual_override_diverter = true;
            gamepad2.rumble(100);
        }
        if (gamepad2.b) {
            gamepad2.rumble(100);
            manual_override_diverter = false;
            gamepad2.rumble(100);
        }
        if (!manual_override_diverter) {
            handle_divert(robotComposition,distance_right,distance_left);
        }
        if (gamepad2.startWasPressed()) {
            is_blue = false;
        }
        if (gamepad2.shareWasPressed()) {
            is_blue = true;
        }
        if (gamepad2.y) {
            gamepad2.rumble(100);
            headingLock = true;
            if (is_blue) {
                targetHeading = Math.toRadians(Limelight.target_apriltag(robotComposition,20));
            } else {
                targetHeading = Math.toRadians(Limelight.target_apriltag(robotComposition,24));
            }
        }
        if (gamepad2.x) {
            gamepad2.setLedColor(1,0,0,100);
            headingLock = false;
        }

//        if (gamepad2.startWasPressed()) {
//            is_configuring = true;
//        }
//        if (gamepad2.shareWasPressed()) {
//            is_configuring = false;
//        }
//        if (is_configuring) {
//            telemetry.addLine("CURRENTLY CONFIGURING!");
//            if (gamepad2.aWasPressed()) {
//                if (diverter.getDirection() == Servo.Direction.FORWARD) {
//                    diverter.setDirection(Servo.Direction.REVERSE);
//                } else {
//                    diverter.setDirection(Servo.Direction.FORWARD);
//                }
//            }
//            if (gamepad2.bWasPressed()) {
//                if (LEFT_POSITION == 0.2962) {
//                    LEFT_POSITION = 1;
//                } else {
//                    LEFT_POSITION = 0.2962;
//                }
//            }
//            if (gamepad2.xWasPressed()) {
//                if (RIGHT_POSITION == 0) {
//                    RIGHT_POSITION = 1;
//                } else {
//                    RIGHT_POSITION = 0;
//                }
//            }
//        } else {
//            handle_divert(robotComposition,distance_left,distance_right);
//        }
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
            if (last_dis_change + dist_wait <= System.currentTimeMillis()) {
                if (launcherDistance == Launcher.LauncherDistance.FAR) {
                    gamepad1.rumble(100);
                    launcherDistance = Launcher.LauncherDistance.CLOSE;
                    launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                    gamepad1.rumble(100);
                } else {
                    gamepad1.rumble(100);
                    launcherDistance = Launcher.LauncherDistance.FAR;
                    launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                }
                last_dis_change = System.currentTimeMillis();
            }
        }
        mecanumDrive();
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
        if (gamepad2.y) {
            relocalize();
        }
        if (gamepad1.yWasPressed()) {
            leftLauncher.setVelocity(launcherTarget);
            rightLauncher.setVelocity(launcherTarget);
        } else if (gamepad1.bWasPressed()) {
            // stop flywheel
            leftLauncher.setVelocity(STOP_SPEED);
            rightLauncher.setVelocity(STOP_SPEED);
        }
        if (gamepad1.xWasPressed()) {
            intakeState = IntakeState.OFF;
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
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
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
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
            Launcher.spam(robotComposition,launcherDistance,0);
            gamepad1.rumble(100);
        }


        /*
         * Show the state and motor powers
         */
        if (is_configuring) {
            if (limelight.getLatestResult().isValid() && limelight_detection_enabled) {
                relocalize();
            }
            panelsTelemetry.addLine("Baller");
            panelsTelemetry.addData("DIVERTER POSITION", diverter.getPosition());
            panelsTelemetry.addData("Left Velocity", leftLauncher.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Right Velocity", rightLauncher.getVelocity(AngleUnit.DEGREES));
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.addData("DISTANCE TO GOAL", Launcher.distance_to_goal(robotComposition,true));
            panelsTelemetry.addData("State", leftLaunchState);
            panelsTelemetry.addData("launch distance", launcherDistance);
            panelsTelemetry.addData("Left Launcher Velocity", leftLauncher.getVelocity());
            panelsTelemetry.addData("Right Launcher Velocity", rightLauncher.getVelocity());
            panelsTelemetry.addData("Loop Time", looptime.milliseconds());
        }

        panelsTelemetry.update();
        panelsTelemetry.update(telemetry);
        looptime.reset();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    public static double turn_multipler = 0.8;

    public void relocalize() {
            // First, tell Limelight which way your robot is facing
            Pose pose = new Pose();
            LLResult result = limelight.getLatestResult();

            double robotYaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.RADIANS).firstAngle;
            limelight.updateRobotOrientation(robotYaw);
            telemetry.addData("Is Result Valid", result.isValid());
            for (LLResultTypes.FiducialResult fiducialResult : limelight.getLatestResult().getFiducialResults()) {
                Pose3D poser = fiducialResult.getRobotPoseFieldSpace();
                telemetry.addData("X", poser.getPosition().x);
                telemetry.addData("Y", poser.getPosition().y);

            }
            if (result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    double heading = botpose_mt2.getOrientation().getYaw();
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    pose = getRobotPoseFromCamera(x, y,heading);
                }
            }
            follower.setPose(pose);
    }
    private Pose getRobotPoseFromCamera(double x, double y,double heading) {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        return new Pose(x, y, heading, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
    public double getHeadingError() {
        return MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
    }

    void mecanumDrive() {
        double modifier = (1 - gamepad1.right_trigger);
        // minimum modifier is 0.25
        if (modifier <= 0.25) {modifier = 0.25;}
        controller.setCoefficients(follower.constants.getCoefficientsHeadingPIDF());
        controller.updateError(getHeadingError());
        telemetry.addData("Heading Error", controller.getError());
        if (headingLock) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run() + -gamepad1.right_stick_x);
            telemetry.addData("Locking Heading", controller.getTargetPosition());
        }else{
            telemetry.addData("Not locking heading", -gamepad1.right_stick_x);
            follower.setTeleOpDrive(-gamepad1.left_stick_y * modifier, -gamepad1.left_stick_x * modifier, -gamepad1.right_stick_x * modifier);
    }}






}