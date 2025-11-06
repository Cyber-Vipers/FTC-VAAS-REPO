package org.firstinspires.ftc.teamcode.Competition.opmodes.auto;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Competition.opmodes.Teleop.Competition_Teleop;
import org.firstinspires.ftc.teamcode.Competition.sensors.limelight_thinker;
import org.firstinspires.ftc.teamcode.pedroPathing.*;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntVisualConstants;
import org.firstinspires.ftc.teamcode.tools.Detection;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@Autonomous(name = "Competition Auto", group = "Competition")
public class Competition_Auto extends SelectableOpMode {

    public static Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public Competition_Auto() {
        super("Pick Your Poison", s -> {
            s.folder("Red", l -> {
                l.add("Red A", Red::new);
            });
            s.folder("Blue", l -> {
                l.add("Blue A", Blue_A1::new);
            });
            s.folder("Testing", l -> {
                l.add("Aim Assist", Aim_Assist::new);
            });
        });
    }

    @Override
    public void onSelect() {
        if (follower == null) {
            follower = RuntVisualConstants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = RuntVisualConstants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void onLog(List<String> lines) {}


    /** This creates a full stop of the robot by setting the drive motors to run at 0 power. */
    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }
}
class Red extends OpMode{
    @Override
    public void init() {
        telemetry.addLine("I'm Red Bro");
    }
    @Override
    public void loop() {
    }
}
class Aim_Assist extends OpMode {
    final double FEED_TIME_SECONDS = 1.45; //The feeder servos run this long when a shot is requested.
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

    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;

    // Declare OpMode members.
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
    private LaunchState rightLaunchState;

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

    private enum LauncherDistance {
        CLOSE,
        FAR;
    }
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    public enum path_pos {
        A1,
        A2,
        A3,
        A4,
        end,
    }
    public Blue_A1.path_pos current_path = Blue_A1.path_pos.A1;
    public boolean launched = false;
    public Limelight3A limelight;
    public int obelisk = 0;
    public int launch_counter = 0;
    public limelight_thinker limelight_thinker;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        telemetry.addLine("I'm Blue Bro");
        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;
        limelight_thinker = new limelight_thinker(limelight);
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
        follower = RuntVisualConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        /*
         * Tell the driver that initialization is complete.
         */

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        follower.update();

        find();
    }

    public void find() {
        Detection detect = limelight_thinker.search();
        if (detect.april == 20) {
            follower.turnTo(Math.toRadians(detect.flat_position.getX()));
        }

    }
}
class Blue_A1 extends OpMode{
    final double FEED_TIME_SECONDS = 1.45; //The feeder servos run this long when a shot is requested.
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

    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;

    // Declare OpMode members.
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
    private LaunchState rightLaunchState;

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

    private enum LauncherDistance {
        CLOSE,
        FAR;
    }
    private Follower follower;
    public static Pose startingPose = new Pose(56.5233644859813,8.29906542056074,90); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    public enum path_pos {
        A1,
        A2,
        A3,
        A4,
        end,
    }
    public path_pos current_path = path_pos.A1;
    public boolean launched = false;
    public Limelight3A limelight;
    public int obelisk = 0;
    public int launch_counter = 0;
    public limelight_thinker limelight_thinker;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        telemetry.addLine("I'm Blue Bro");
        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;
        limelight_thinker = new limelight_thinker(limelight);
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
        follower = RuntVisualConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void start() {
        intakeState = IntakeState.ON;
        intake.setPower(1);
    }
    @Override
    public void loop() {
        follower.update();
        telemetry.update();
        Detection detect = limelight_thinker.search();
        if (detect.april != 0) {
            telemetry.addData("Selected Obelisk", detect.april);
            obelisk = detect.april;
        }
        if (!follower.isBusy()) {
            switch (current_path) {
                case A1:
                    launcherDistance = LauncherDistance.FAR;
                    launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                    follower.followPath(new Blue_A1_Paths(follower).Path1);
                    break;
                case A2:
                    follower.followPath(new Blue_A1_Paths(follower).Path2);
                    break;
                case A3:
                    follower.followPath(new Blue_A1_Paths(follower).Path3);
                    break;
                case A4:
                    follower.followPath(new Blue_A1_Paths(follower).Path4);
                    break;
            }
        }
        if (follower.atParametricEnd()) {
            switch (current_path) {
                case A1:
                    current_path = path_pos.A2;
                case A2:

                    current_path = path_pos.A3;
                    break;
                case A3:
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    current_path = path_pos.A4;
                    break;
                case A4:
                    current_path = path_pos.end;
                    break;
                case end:
                    //logic for shooting correct
                    //two purple left one green right
                    if (obelisk == 21) {
                        //green purple purple
                        launchRight(true);
                        if (rightLaunchState == LaunchState.IDLE && !launched) {
                            launched = true;
                            launchLeft(true);
                        }
                        if (rightLaunchState == LaunchState.IDLE && launched) {
                            launchLeft(true);

                        }

                    }
                    if (obelisk == 22) {
                        if (leftLaunchState == LaunchState.IDLE && !launched) {
                            launchLeft(true);
                            launched = true;
                        }
                        if (launched && leftLaunchState == LaunchState.IDLE) {
                            launchRight(true);
                            launched = false;
                        }
                    }
                    if (obelisk == 23) {
                        if (leftLaunchState == LaunchState.IDLE && !launched) {
                            launchLeft(true);
                            launch_counter = 1;
                        } else if (!launched && leftLaunchState == LaunchState.IDLE && launch_counter == 1) {
                            launch_counter = 2;
                            launchRight(true);
                            launched = true;
                        }

                    }
                    break;
            }
            telemetry.addLine("On Path" + current_path.name());
        }
    }

    class Blue_A1_Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Blue_A1_Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.523, 8.299), new Pose(61.682, 16.374))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(61.682, 16.374), new Pose(36.336, 35.439))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.336, 35.439), new Pose(19.514, 35.439))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.514, 35.439), new Pose(61.682, 15.925))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
    }
}