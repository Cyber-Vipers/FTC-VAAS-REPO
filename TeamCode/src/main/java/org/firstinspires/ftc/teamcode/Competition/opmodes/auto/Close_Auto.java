package org.firstinspires.ftc.teamcode.Competition.opmodes.auto;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.opmodes.Teleop.Competition_Teleop;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


@Autonomous(name = "Close Auto")
public class Close_Auto extends OpMode {
    //**PRELOAD MUST BE PURPLE ON RIGHT ONE GREEN ON LEFT**
    public int loops = 0;
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState = LaunchState.IDLE;
    private LaunchState rightLaunchState = LaunchState.IDLE;
    ElapsedTime leftFeederTimer = new ElapsedTime();
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
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo diverter = null;
    private Follower follower;
    public Limelight3A limelight;
    @Override
    public void init() {
        telemetry.addLine("I'm Blue Bro");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        intake = hardwareMap.get(DcMotor.class, "rollerIntake");
        leftFeeder = hardwareMap.get(CRServo.class, "indexLeft");
        rightFeeder = hardwareMap.get(CRServo.class, "indexRight");
        diverter = hardwareMap.get(Servo.class, "modulator");
        telemetry.addLine("new");
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
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        follower = RuntOdometryWheelsConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(127.52329576522467,125.1409210488048,Math.toRadians(35)));
        follower.update();
        Ze_Paths = new Da_Paths(follower);
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }
    public enum paths {
        A1,
        A2,
        A2FIRE,
        A3,
        A4,
        A5,
        A6,
        A6Fire,
        A7,
        A8,
        A9,
        A10,
        end
    }
    public Da_Paths Ze_Paths;
    public paths currentpath = paths.A1;
    public boolean obelisk_changed;
    public int obelisk = 21;
    public long fire_wait = 6000;
    public long last_fire_time = 0;
    public int fire = 0;
    @Override
    public void loop() {
        telemetry.addData("Right Launcher", rightLauncher.getVelocity());
        telemetry.addData("Left Launcher", leftLauncher.getVelocity());
        leftLauncher.setVelocity(launcherTarget);
        rightLauncher.setVelocity(launcherTarget);
        intake.setPower(1);
        follower.update();
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Pose", follower.getPose());
        if ((currentpath != paths.A2FIRE && currentpath != paths.A6Fire && currentpath != paths.end)) {
            telemetry.addData("Conditional", (currentpath != paths.A2FIRE || currentpath != paths.A6Fire || currentpath != paths.end));
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
        }
        switch (currentpath) {
            case A1:
                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
                if (loops == 0) {
                    follower.followPath(Ze_Paths.Path1);
                    currentpath = paths.A2;
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
            case A2:
                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path2);
                    currentpath = paths.A2FIRE;
                }
                break;
            case A2FIRE:
                //purple on right green on left
                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
                if (follower.atParametricEnd()) {
                if (fire == 0) {
                    last_fire_time = System.currentTimeMillis();
                    fire = 1;
                }
                if (last_fire_time + (fire_wait - (5000)) <= System.currentTimeMillis()) {
                    switch (obelisk) {
                        case 21:
                            if (last_fire_time + (fire_wait * 0.35)  <= System.currentTimeMillis()) {
                                launchLeft(true);
                            }
                            if (last_fire_time + (fire_wait * 0.55) <= System.currentTimeMillis()) {
                                launchRight(true);
                            }
                            if (last_fire_time + (fire_wait * 0.65) <= System.currentTimeMillis()) {

                                launchRight(true);
                            }
                            break;
                        case 22:
                            if (last_fire_time + (fire_wait * 0.35)  <= System.currentTimeMillis()) {
                                launchLeft(true);
                            }
                            if (last_fire_time + (fire_wait * 0.55) <= System.currentTimeMillis()) {
                                launchRight(true);
                            }
                            if (last_fire_time + (fire_wait * 0.65) <= System.currentTimeMillis()) {

                                launchLeft(true);
                            }
                            break;
                        case 23:
                            if (last_fire_time + (fire_wait * 0.35)  <= System.currentTimeMillis()) {
                                launchRight(true);
                            }
                            if (last_fire_time + (fire_wait * 0.55) <= System.currentTimeMillis()) {
                                launchRight(true);
                            }
                            if (last_fire_time + (fire_wait * 0.65) <= System.currentTimeMillis()) {

                                launchLeft(true);
                            }
                            break;
                    }
                }

                if (last_fire_time + fire_wait <= System.currentTimeMillis()) {
                    currentpath = paths.A3;
                }

                }
                break;
            case A3:

                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path3);
                    currentpath = paths.A4;
                }
                break;
            case A4:

                if (follower.atParametricEnd()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(Ze_Paths.Path4);
                    currentpath = paths.A5;

                }
            case A5:
                diverterDirection = DiverterDirection.LEFT;
                diverter.setPosition(LEFT_POSITION);
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path5);
                    currentpath = paths.A6;
                }
            case A6:
                if (follower.atParametricEnd()) {
                    follower.setMaxPower(1);
                    follower.followPath(Ze_Paths.Path6);
                    currentpath = paths.A6Fire;
                }
            case A6Fire:
                //wait 3 seconds
                if (follower.atParametricEnd()) {
                if (fire == 1) {
                    last_fire_time = System.currentTimeMillis();
                    fire = 2;
                }
                    switch (obelisk) {
                        case 21:
                            if (last_fire_time + (fire_wait * 0.35)  <= System.currentTimeMillis()) {
                                launchLeft(true);
                            }
                            if (last_fire_time + (fire_wait * 0.55) <= System.currentTimeMillis()) {
                                launchRight(true);
                            }
                            if (last_fire_time + (fire_wait * 0.65) <= System.currentTimeMillis()) {

                                launchRight(true);
                            }
                            break;
                        case 22:
                            if (last_fire_time + (fire_wait * 0.35)  <= System.currentTimeMillis()) {
                                launchLeft(true);
                            }
                            if (last_fire_time + (fire_wait * 0.55) <= System.currentTimeMillis()) {
                                launchRight(true);
                            }
                            if (last_fire_time + (fire_wait * 0.65) <= System.currentTimeMillis()) {

                                launchLeft(true);
                            }
                            break;
                        case 23:
                            if (last_fire_time + (fire_wait * 0.35)  <= System.currentTimeMillis()) {
                                launchRight(true);
                            }
                            if (last_fire_time + (fire_wait * 0.55) <= System.currentTimeMillis()) {
                                launchRight(true);
                            }
                            if (last_fire_time + (fire_wait * 0.65) <= System.currentTimeMillis()) {

                                launchLeft(true);
                            }
                            break;
                    }
                if (last_fire_time + fire_wait <= System.currentTimeMillis()) {
                    currentpath = paths.A7;
                }
                }
                break;
            case A7:
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path7);
                    currentpath = paths.A8;
                }
            case A8:
                if (follower.atParametricEnd()) {
                    follower.setMaxPower(0.5);
                        follower.followPath(Ze_Paths.Path8);
                        currentpath = paths.A9;
                }
            case A9:
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path9);
                    currentpath = paths.A10;
                }
            case A10:
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path10);
                    follower.setMaxPower(1);
                    currentpath = paths.end;
                }
            case end:
                //wait 3 seconds
                if (follower.atParametricEnd()) {
                    if (fire == 1) {
                        last_fire_time = System.currentTimeMillis();
                        fire = 2;
                    }
                    if (last_fire_time + fire_wait <= System.currentTimeMillis()) {
                        currentpath = paths.end;
                    }
                }
                break;
        }
    }

    public enum close_state {
        A1,
        A2
    }



    public static class Da_Paths {
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
        public Da_Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(127.523, 125.141), new Pose(109.562, 106.057))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(35))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(109.562, 106.057), new Pose(92.499, 125))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(7))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(92.499, 125), new Pose(101.480, 78.667))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.480, 78.667), new Pose(114.502, 79.116))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(114.502, 79.116), new Pose(130.442, 80.014))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130.442, 80.014), new Pose(92.499, 133.223))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(92.499, 133.223), new Pose(101.480, 55.093))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.480, 55.093), new Pose(113.379, 55.542))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(113.379, 55.542), new Pose(124.605, 55.542))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(124.605, 55.542), new Pose(92.499, 133.223))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
        }
    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {
            case IDLE:
                if (shotRequested) {
                    leftLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                if (leftLauncher.getVelocity() > launcherMin) {
                    leftLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(-FULL_SPEED);
                leftFeederTimer.reset();
                leftLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftLaunchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

    void launchRight(boolean shotRequested) {
        switch (rightLaunchState) {
            case IDLE:
                if (shotRequested) {
                    rightLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                if (leftLauncher.getVelocity() > launcherMin) {
                    rightLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                rightFeeder.setPower(-FULL_SPEED);
                rightFeederTimer.reset();
                rightLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rightLaunchState = LaunchState.IDLE;
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }}


