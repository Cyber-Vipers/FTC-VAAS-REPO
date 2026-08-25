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

import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher.distance_to_goal;
import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher.launchLeft;
import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher.launchRight;
import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher.linearRegression;
import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Sphere_Operator.force_divert;
import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Sphere_Operator.handle_divert;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.field.Style;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Util.Robot_Composition;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Sphere_Operator;

import org.firstinspires.ftc.teamcode.Robot_Competition_20525.opmodes.auto.BlueCloseAutoIteration2;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.opmodes.auto.BlueFarAutoIteration2;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.opmodes.auto.RedCloseAutoIteration2;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.opmodes.auto.RedFarAutoIteration2;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;
import org.firstinspires.ftc.teamcode.util.BangBangController;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */
@Disabled
@Configurable
@TeleOp(name = "Competition Teleop", group = "Comp" )
//@Disabled
public class Competition_Teleop extends OpMode {
    public static PIDFCoefficients flywheel_tuning = new PIDFCoefficients(1000, 0, 0, 2.5);
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    static Servo servo;
    public enum ServoPos {
        Up(0),
        Down(0.5);
        private double value;
        ServoPos(double value) {
            this.value = value;
        }
        public void move() {
            servo.setPosition(value);
        }
    }
    public ServoPos currentPos = ServoPos.Up;
    public void blah() {
        currentPos = ServoPos.Down;


        currentPos.move();
    }


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
    public static Pose startingPose = new Pose(9.5,9.5,Math.toRadians(90)); //See ExampleAuto to understand how to use this
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
    public static double f = 0.002;
//    public static double override_value = 1200;
    public static enum LastAutoStates {
        BlueClose,
        BlueFar,
        RedFar,
        RedClose,
        None,
        Teleop
    }
    public static LastAutoStates lastauto = LastAutoStates.None;
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
        leftFeeder.setDirection(Servo.Direction.REVERSE);
        rightFeeder.setDirection(Servo.Direction.REVERSE);
        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may requirgae direction flips
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
        limelight.setPollRateHz(100);
        limelight.start();
        PIDFController controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        boolean headingLock = true;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("This is the TELEOP for 20525 for Config 'Silly Robot'"," This Does NOT MOVE (off the table) without input");
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsField.init();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        panelsField.setStyle("red", "blue", 2.0);//fill, outline fill, outline width
        follower.setPose(startingPose);
        lastauto = LastAutoStates.Teleop;

        Launcher.auto_active = true;
        Launcher.noshoot = false;
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
//        Launcher.shut_gates(robotComposition);
//        Launcher.spinup(robotComposition,launcherDistance,0);
//        Limelight.look_up(robotComposition);
        //)

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
    public boolean is_blue = true;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    public static boolean is_configuring = true;
    public static Pose blueGoalPosition = new Pose(14,130);
    public static Pose redGoalPosition = new Pose(130,130);
    public static Pose goalPosition = new Pose(14,130);
    public static double regression_output = 0;
//    public static boolean regression_active = true;


    public static enum RegressionMode {
        Quadratic,
    }

    public static Pose EndingPose = new Pose(0,0);

    public static RegressionMode regressionMode = RegressionMode.Quadratic;
    public void draw(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.setFill("");
        panelsField.circle(84);
        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * 9);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();
        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
        sendPacket();
    }
    public boolean human_loading = false;

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.0
    );
    public static void sendPacket() {
        panelsField.update();
    }
    @Override
    public void loop() {
        looptime.startTime();
        EndingPose = follower.getPose();
        draw(follower.getPose(), robotLook);
        telemetry.addData("CONFIG", is_configuring);
        telemetry.addData("Left Flywheel", Math.round(leftLauncher.getVelocity()));
        panelsField.update();
        follower.update();
        telemetry.addData("Distance to goal", Math.round(distance_to_goal(follower.getPose(), goalPosition)));
        double output = Math.max(Math.min(BangBangController.BangBangChicken(rightLauncher.getVelocity(), 1300d + 25d, f, 1), 1), -1);
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_tuning);
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_tuning);
        double distance_left = colorleft.getDistance(DistanceUnit.CM);
        double distance_right = colorright.getDistance(DistanceUnit.CM);
        panelsField.moveCursor(follower.getPose().getX(), follower.getPose().getY());
//        if (headingLock) {
//            if (is_blue) {
//                targetHeading = Math.toRadians(Limelight.target_apriltag(robotComposition,20));
//            } else {
//                targetHeading = Math.toRadians(Limelight.target_apriltag(robotComposition,24));
//            }
//        }
        if (rightLauncher.getVelocity() >= Launcher.launcherTarget) {
            gamepad1.setLedColor(0, 1, 0, 100);
        }
        if (gamepad2.a) {
            gamepad2.rumble(100);
            manual_override_diverter = false;
        }
        if (gamepad2.b) {
            gamepad2.rumble(100);
            manual_override_diverter = false;
            gamepad2.rumble(100);
        }
        if (!manual_override_diverter) {
            handle_divert(robotComposition, distance_left, distance_right);
        }
        if (gamepad1.startWasPressed()) {
            is_blue = false;
        }
        if (gamepad1.shareWasPressed()) {
            is_blue = true;

        }
//        if (gamepad1.leftStickButtonWasPressed()) {
//            regression_active = !regression_active;
//            is_configuring = !is_configuring;
//        }

        if (is_blue == true) {
            goalPosition = blueGoalPosition;
        } else {
            goalPosition = redGoalPosition;
        }
//        if (gamepad1.dpad_down) {
//            relocalize();
//        }
//        if (gamepad2.right_stick_button) {
//            if (headingLock && last_heading_swap + heading_cooldown < System.currentTimeMillis()) {
//                gamepad2.rumble(100);
//                gamepad2.rumble(100);
//                headingLock = false;
//            } else {
//                gamepad2.rumble(100);
//                headingLock = true;
//                if (is_blue) {
//                    targetHeading = Math.toRadians(Limelight.target_apriltag(robotComposition,20));
//                } else {
//                    targetHeading = Math.toRadians(Limelight.target_apriltag(robotComposition,24));
//                }
//            }
//            switch (lastauto) {
//                case RedClose:
//                    is_blue = false;
//                    if (RedCloseAutoIteration2.EndingPose != null) {
//                        EndingPose = RedCloseAutoIteration2.EndingPose;
//                    }
//
//                    break;
//                case RedFar:
//                    is_blue = false;
//                    follower.setPose(RedFarAutoIteration2.EndingPose);
//                    break;
//                case BlueClose:
//                    if (BlueCloseAutoIteration2.EndingPose != null) {
//                        EndingPose = BlueCloseAutoIteration2.EndingPose;
//                    }
//                    break;
//                case BlueFar:
//                    follower.setPose(BlueFarAutoIteration2.EndingPose);
//                    break;
//                case None:
//                    regression_active = false;
//                    follower.setPose(startingPose);
//                case Teleop:
//                    follower.setPose(EndingPose);
//            }
//        }


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
        if (gamepad2.dpadDownWasPressed()) {
            Launcher.open_gates(robotComposition);
        }
        if (gamepad2.dpadLeftWasPressed()) {
            Launcher.launchLeft(true,robotComposition,launcherDistance,0);
        }
        if (gamepad2.dpadRightWasPressed()) {
            Launcher.launchRight(true,robotComposition,launcherDistance,0);
        }
        if (gamepad1.rightBumperWasPressed()) {
            if (last_dis_change + dist_wait <= System.currentTimeMillis()) {
                if (launcherDistance == Launcher.LauncherDistance.FAR) {
                    gamepad1.rumble(100);
                    launcherDistance = Launcher.LauncherDistance.CLOSE;
                    launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
//                    override_value = 1250;
                    launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                    gamepad1.rumble(100);
                } else {
                    gamepad1.rumble(100);
                    launcherDistance = Launcher.LauncherDistance.FAR;
                    launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
//                    override_value = 1350;
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
        // HUMAN PLAYER LOAD
        //telemetry.addData("RIGHT LAUNCHER DIRECTION", rightLauncher.getDirection());
        if (gamepad2.y) {
//            regression_active = false;
        }
        if (gamepad2.dpadUpWasPressed()) {
            if (!human_loading) {
                leftLauncher.setVelocity(-launcherTarget);
                rightLauncher.setVelocity(-launcherTarget);
                human_loading = true;
                Launcher.open_gates(robotComposition);
            } else {
                Launcher.shut_gates(robotComposition);
                leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
                rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                human_loading = false;
            }
        }

        if (gamepad1.yWasPressed()) {
            human_loading = false;
            leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE   );
            rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            leftLauncher.setVelocity(launcherTarget);
            rightLauncher.setVelocity(launcherTarget);
//            if (regression_active) {
//                is_configuring = true;
//                switch (regressionMode) {
//                    case Quadratic:
//                        override_value = linearRegression(follower.getPose(),goalPosition);
//                        break;
//                }
//            }

        } else if (gamepad1.bWasPressed()) {
            // stop flywheel
            human_loading = false;
            leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE   );
            rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            leftLauncher.setVelocity(STOP_SPEED);
            rightLauncher.setVelocity(STOP_SPEED);
        }
        if (gamepad1.rightStickButtonWasPressed()) {
            if (!is_blue){
                gamepad1.rumble(100);
                follower.setPose(new Pose(134,9,90));


            } else {
                gamepad1.rumble(100);
                follower.setPose(new Pose(9,9,90));
            }

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
        if (gamepad1.leftBumperWasPressed()) {
            if (last_divert_time + divert_cooldown < System.currentTimeMillis()) {
                switch (diverterDirection) {
                    case LEFT:
                        diverterDirection = DiverterDirection.RIGHT;
                        force_divert(robotComposition,false);
                        break;
                    case RIGHT:
                        diverterDirection = DiverterDirection.LEFT;
                        force_divert(robotComposition,true);
                        break;
                }
                last_divert_time = System.currentTimeMillis();
            }
        }



        /*
         * Now we call our "Launch" function.
         */

//        if (regression_active) {
//            is_configuring = true;
//            override_value = linearRegression(follower.getPose(),goalPosition);
//            regression_output = override_value;
//        }

        //telemetry.addData("Regression Output", regression_output);
        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
            human_loading = false;
        }

        launchLeft(gamepad1.dpad_left,robotComposition,launcherDistance,0);
        launchRight(gamepad1.dpad_right,robotComposition,launcherDistance,0);
        if (gamepad1.dpad_up) {
//            leftLauncher.setVelocity(override_value);
//            rightLauncher.setVelocity(override_value);
            launchLeft(gamepad1.dpad_up,robotComposition,launcherDistance,0);
            launchRight(gamepad1.dpad_up,robotComposition,launcherDistance,0);
            gamepad1.rumble(100);
        }

//        telemetry.addData("OVERRIDE", override_value);
        /*
         * Show the state and motor powers
         */
//        if (is_configuring) {
//            if (limelight.getLatestResult().isValid() && limelight_detection_enabled) {
//                relocalize();
//            }
//            //telemetry.addData("Right Velocity", rightLauncher.getVelocity());
//            //telemetry.addData("Loop Time", looptime.milliseconds());
//        }

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
            LLResult result = limelight.getLatestResult();

            //telemetry.addData("Is Result Valid", result.isValid());
            for (LLResultTypes.FiducialResult fiducialResult : limelight.getLatestResult().getFiducialResults()) {
                Pose3D poser = fiducialResult.getRobotPoseFieldSpace();
                //telemetry.addData("X", poser.getPosition().x);
                //telemetry.addData("Y", poser.getPosition().y);
                if (result.isValid()) {
                    double x = poser.getPosition().x;
                    double y = poser.getPosition().y;
                    //telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    Pose2D apriltag = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, follower.getHeading()); // Likely given by an AprilTagProcessor or Limelight instead
                    Pose ftcStandard = PoseConverter.pose2DToPose(apriltag, InvertedFTCCoordinates.INSTANCE);
                    Pose pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                    // If we want to set our pose, to what we were given by the Pose2D, we can add:
                    follower.setPose(pedroPose);
                }
            }



    }
    private Pose getRobotPoseFromCamera(double x, double y,double heading) {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        Pose thought_pose = new Pose(x, y, heading, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        return thought_pose;
    }
    public double getHeadingError() {
        return MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
    }






    void mecanumDrive() {
        double modifier = (1 - gamepad1.right_trigger);
        // minimum modifier is 0.25
        if (modifier <= 0.25) {modifier = 0.25;}
        controller.setCoefficients(follower.constants.getCoefficientsHeadingPIDF());
        double head = BangBangController.BangBangChicken(follower.getHeading(),targetHeading,f,true);
        controller.updateError(getHeadingError());
        //telemetry.addData("Current Heading", Math.toDegrees(follower.getHeading()));
        //telemetry.addData("Target Heading", Math.toDegrees(targetHeading));
        controller.updateError(getHeadingError());
        //telemetry.addData("Heading Error", controller.getError());
        if (headingLock) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, head + -gamepad1.right_stick_x);
            //telemetry.addData("Locking Heading", head);
        }else{
            telemetry.addData("Not locking heading", -gamepad1.right_stick_x);
            follower.setTeleOpDrive(-gamepad1.left_stick_y * modifier, -gamepad1.left_stick_x * modifier, -gamepad1.right_stick_x * modifier);
    }}






}