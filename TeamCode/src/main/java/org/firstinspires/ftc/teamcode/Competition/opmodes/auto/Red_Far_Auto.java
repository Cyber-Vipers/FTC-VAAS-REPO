package org.firstinspires.ftc.teamcode.Competition.opmodes.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher.launchLeft;
import static org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher.launchRight;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Robot_Composition;
import org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Competition.Subsystems.Sphere_Operator;

import java.util.ArrayList;


@Configurable
public class Red_Far_Auto extends OpMode {
    //**PRELOAD MUST BE PURPLE ON RIGHT ONE GREEN ON LEFT**
    //TODO actually add logic, shooting, etc
    public int loops = 0;
    public static double left_feeder_block = 0.25;
    public static double right_feeder_block = 0.25;
    public static double right_feeder_up = 0;
    public static double left_feeder_up = 1;
    public DcMotor feeder;
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState = LaunchState.IDLE;
    private LaunchState rightLaunchState = LaunchState.IDLE;
    ElapsedTime leftFeederTimer = new ElapsedTime();
    Robot_Composition robotComposition;
    Rev2mDistanceSensor color1;
    Rev2mDistanceSensor color2;
    ElapsedTime rightFeederTimer = new ElapsedTime();
    final double FEED_TIME_SECONDS = 1.45; //The feeder servos run this long when a shot is requested.
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1150; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1130; //minimum required to start a shot for close goal.
    private boolean slowMode = false;
    private boolean correct_me = false;
    private double slowModeMultiplier = 0.5;
    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;

    // Declare OpMode members.
    private enum DiverterDirection {
        LEFT,
        RIGHT;
    }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private DcMotor intake = null;
    private Servo leftFeeder = null;
    private Servo rightFeeder = null;
    private Servo diverter = null;
    private Follower follower;
    public Limelight3A limelight;
    public Servo limelight_servo;
    @Override
    public void init() {
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
        color1 = hardwareMap.get(Rev2mDistanceSensor.class,"color1");
        color2 = hardwareMap.get(Rev2mDistanceSensor.class,"color2");
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

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

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

        follower.setStartingPose(new Pose(64,8,Math.toRadians(90)).mirror());
        follower.update();
        Ze_Paths = new Da_Paths(follower);
        robotComposition = new Robot_Composition(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, leftLauncher, rightLauncher, intake, feeder, leftFeeder, rightFeeder,color1,color2,follower,telemetry,diverter,limelight_servo,limelight);
        // if preload set it
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
        robotComposition.right_inside = right_inside;
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("This is the concept for the far auto for 20525 with config 'Silly Robot' ", "This DOES move (off the table)");
        telemetry.addData("Status", "Initialized");
    }
    public enum paths {
        Path1,
        Path1Fire,
        Path2,
        Path2FIRE,
        Path3,
        Path4,
        Path5,
        Path5Fire,
        Path6,
        Path6Fire,
        Path7,
        Path8,
        Path9,
        Path9Fire,
        Path10,
        Path11,
        Path12,
        Path12Fire,
        Path13,
        end
    }
    public Da_Paths Ze_Paths;
    public paths currentpath = paths.Path1;
    public boolean obelisk_changed;
    public int obelisk = 21;
    public static long fire_wait = 6000;
    public long last_fire_time = 0;
    public int fire = 0;

    @Override
    public void loop() {
        if (currentpath != paths.Path1Fire && currentpath != paths.Path5Fire && currentpath != paths.Path9Fire && currentpath != paths.Path12Fire) {
            feeder.setPower(0);
        }
        intake.setPower(1);
        switch (currentpath) {
            case Path1:
                //Divert to right, follow path 1, then read obelisk
                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
                if (loops == 0) {
                    follower.followPath(Ze_Paths.Path1);
                    currentpath = paths.Path2;
                }
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = result.getTa();
                    for (LLResultTypes.FiducialResult april : result.getFiducialResults()) {
                        if (april.getFiducialId() != 20 & april.getFiducialId() != 24) {
                            obelisk = april.getFiducialId();
                            obelisk_changed = true;
                        }
                    }
                }
                break;
            case Path1Fire:
                //TODO fire
                if (fire == 0) {
                    last_fire_time = System.currentTimeMillis();
                    fire = 1;
                }
                if (last_fire_time + (fire_wait - (5000)) <= System.currentTimeMillis()) {
                    switch (obelisk) {
                        case 21:
                            //Left Right Right
                            // aka Green Purple Purple
                            if (last_fire_time + (fire_wait * 0.35)  <= System.currentTimeMillis()) {
                                launchLeft(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                            }
                            if (last_fire_time + (fire_wait * 0.55) <= System.currentTimeMillis()) {
                                launchRight(true,robotComposition,Launcher.LauncherDistance.FAR,0);
                            }
                            if (last_fire_time + (fire_wait * 0.65) <= System.currentTimeMillis()) {

                                launchRight(true,robotComposition,Launcher.LauncherDistance.FAR,0);
                            }
                            break;
                        case 22:
                            // Right Left Right
                            // aka Purple Green Purple
                            if (last_fire_time + (fire_wait * 0.35)  <= System.currentTimeMillis()) {
                                launchRight(true,robotComposition,Launcher.LauncherDistance.FAR,0);
                            }
                            if (last_fire_time + (fire_wait * 0.55) <= System.currentTimeMillis()) {
                                launchLeft(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                            }
                            if (last_fire_time + (fire_wait * 0.65) <= System.currentTimeMillis()) {

                                launchRight(true,robotComposition,Launcher.LauncherDistance.FAR,0);
                            }
                            break;
                        case 23:
                            // Right Right Left
                            // aka Purple Purple Green 😎
                            if (last_fire_time + (fire_wait * 0.35)  <= System.currentTimeMillis()) {
                                launchRight(true,robotComposition,Launcher.LauncherDistance.FAR,0);
                            }
                            if (last_fire_time + (fire_wait * 0.55) <= System.currentTimeMillis()) {
                                launchRight(true,robotComposition,Launcher.LauncherDistance.FAR,0);
                            }
                            if (last_fire_time + (fire_wait * 0.65) <= System.currentTimeMillis()) {

                                launchLeft(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                            }
                            break;
                    }
                }
                if (last_fire_time + fire_wait <= System.currentTimeMillis()) {

                    currentpath = paths.Path2;
                    follower.followPath(Ze_Paths.Path2);
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                }
            case Path2:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path3;
                    follower.followPath(Ze_Paths.Path3);
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                }
            case Path3:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path4;
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    follower.followPath(Ze_Paths.Path4);
                }
            case Path4:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path5;
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    follower.followPath(Ze_Paths.Path5);
                }
            case Path5:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path5Fire;
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    follower.followPath(Ze_Paths.Path6);
                }
            case Path5Fire:
                //TODO fire
                if (fire == 1){
                    last_fire_time = System.currentTimeMillis();
                    fire = 2;
                }
                if(last_fire_time + fire_wait < System.currentTimeMillis()) {
                    currentpath = paths.Path6;
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    follower.followPath(Ze_Paths.Path6);
                }
            case Path6:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path7;
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                    follower.followPath(Ze_Paths.Path7);
                }
            case Path7:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path8;
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                    follower.followPath(Ze_Paths.Path8);
                }
            case Path8:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path9;
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                    follower.followPath(Ze_Paths.Path9);
                }
            case Path9:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path9Fire;
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                    follower.followPath(Ze_Paths.Path9);
                }
            case Path9Fire:
                //TODO fire
                if (fire == 2){
                    last_fire_time = System.currentTimeMillis();
                    fire = 3;
                }
                if(last_fire_time + fire_wait <= System.currentTimeMillis()) {
                    currentpath = paths.Path10;
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                    follower.followPath(Ze_Paths.Path10);
                }
            case Path10:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path11;
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    follower.followPath(Ze_Paths.Path11);
                }
            case Path11:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path12;
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    follower.followPath(Ze_Paths.Path12);
                }
            case Path12:
                if (follower.atParametricEnd()) {
                    currentpath = paths.Path12Fire;
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    follower.followPath(Ze_Paths.Path12);
                }
            case Path12Fire:
                //TODO fire
                if (fire == 3) {
                    last_fire_time = System.currentTimeMillis();
                    fire = 4;
                }
                if (last_fire_time + fire_wait <= System.currentTimeMillis()) {
                    currentpath = paths.Path13;
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    follower.followPath(Ze_Paths.Path13);
                }
            case Path13:
                if (follower.atParametricEnd()) {
                    currentpath = paths.end;
                }
        }
    }
    private class Da_Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        Da_Paths (Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(64.000, 8.000).mirror(), new Pose(70.206, 10.991).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(70.206, 10.991).mirror(), new Pose(41.271, 36.112).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.271, 36.112).mirror(), new Pose(36.561, 35.888).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.561, 35.888).mirror(), new Pose(21.533, 35.888).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.533, 35.888).mirror(), new Pose(72.449, 83.664).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.449, 83.664).mirror(), new Pose(50.916, 59.439).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.916, 59.439).mirror(), new Pose(36.785, 58.991).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.785, 58.991).mirror(), new Pose(17.495, 58.542).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.495, 58.542), new Pose(72.449, 83.664))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.449, 83.664).mirror(), new Pose(39.701, 83.439).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.701, 83.439).mirror(), new Pose(18.841, 83.215).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.841, 83.215).mirror(), new Pose(72.449, 83.664).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            Path13 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.449, 83.664).mirror(), new Pose(64.000, 8.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(90))
                    .build();
        }
        }

    }
