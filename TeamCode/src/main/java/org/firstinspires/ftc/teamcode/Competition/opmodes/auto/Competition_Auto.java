package org.firstinspires.ftc.teamcode.Competition.opmodes.auto;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.fasterxml.jackson.core.JsonFactory;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Competition.opmodes.Teleop.Competition_Teleop;
import org.firstinspires.ftc.teamcode.Competition.sensors.limelight_thinker;
import org.firstinspires.ftc.teamcode.pedroPathing.*;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntVisualConstants;
import org.firstinspires.ftc.teamcode.tools.Detection;
import org.json.JSONArray;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

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
                l.add("Blue A", Blue_Auto_1::new);
            });
            s.folder("Testing", l -> {
                l.add("Aim Assist", Aim_Assist::new);
                l.add("Testing Blue A1", Testing_Blue_A1::new);
            });
        });
    }
    Limelight3A limelight;
    @Override
    public void onSelect() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
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
class JAuto extends OpMode{
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
    private JAuto.LaunchState leftLaunchState;
    private JAuto.LaunchState rightLaunchState;

    private enum DiverterDirection {
        LEFT,
        RIGHT;
    }
    private JAuto.DiverterDirection diverterDirection = JAuto.DiverterDirection.LEFT;

    private enum IntakeState {
        ON,
        OFF;
    }

    private JAuto.IntakeState intakeState = JAuto.IntakeState.OFF;

    private enum LauncherDistance {
        CLOSE,
        FAR;
    }
    private Follower follower;
    public static Pose startingPose = new Pose(56.5233644859813,8.29906542056074,90); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private JAuto.LauncherDistance launcherDistance = JAuto.LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    public int last_action = 0;
    public enum path_pos {
        Start,
        FireStart,
        A1,
        A2,
        A3,
        A4,
        A4Fire,
        A5,
        A6,
        A7,
        A8,
        A9,
        end,
    }
    public JAuto.path_pos current_path = JAuto.path_pos.Start;
    public boolean launched = false;
    public Limelight3A limelight;
    public int obelisk = 21;
    public int launch_counter = 0;
    public long last_fire_time = 0;
    public limelight_thinker limelight_thinker;
    public boolean has_diverted = false;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        telemetry.addLine("I'm Blue Bro");
        leftLaunchState = JAuto.LaunchState.IDLE;
        rightLaunchState = JAuto.LaunchState.IDLE;
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
        follower = RuntVisualConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }
    public int fire = 0;
    public static final double ROBOT_RADIUS = 9; // woah

    public long fire_pause = 7000;
    public boolean obelisk_changed = false;
    public long divert_time = 0;
    public long divert_wait = 1000;
    JAuto paths;
    @Override
    public void start() {

        intakeState = JAuto.IntakeState.ON;
        diverterDirection = JAuto.DiverterDirection.LEFT;
        diverter.setPosition(LEFT_POSITION);
        intake.setPower(1);
    }


    @Override
    public void loop(){
        follower.update();
        telemetry.update();
        telemetry.addData("Current ", current_path.name());
        telemetry.addData("Fire condition ", System.currentTimeMillis() + fire_pause > last_fire_time);
        telemetry.addData("Divert condition ", System.currentTimeMillis() + divert_wait > divert_time);
        telemetry.addData("Divert Position ", diverterDirection.name());
        telemetry.addData("Intake State ", intakeState.name());
        telemetry.addData("Follower Busy? ", follower.isBusy());
        telemetry.addData("Obelisk ", obelisk);
        telemetry.addData("Obelisk Changed? ", obelisk_changed);
        if (!follower.isBusy()) {
            switch (current_path) {

            }
        }
    }
    public class JAuto_Paths {


    }
}

class CarpI1 extends OpMode{
    //**PRELOAD MUST BE PURPLE ON RIGHT ONE GREEN ON LEFT**
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


    enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private Blue_Auto_1.LaunchState leftLaunchState;
    private Blue_Auto_1.LaunchState rightLaunchState;

    enum DiverterDirection {
        LEFT,
        RIGHT;
    }
    private Blue_Auto_1.DiverterDirection diverterDirection = Blue_Auto_1.DiverterDirection.LEFT;

    enum IntakeState {
        ON,
        OFF;
    }

    private Blue_Auto_1.IntakeState intakeState = Blue_Auto_1.IntakeState.OFF;

    enum LauncherDistance {
        CLOSE,
        FAR;
    }
    private Follower follower;
    public static Pose startingPose = new Pose(56.5233644859813,8.29906542056074,90); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Blue_Auto_1.LauncherDistance launcherDistance = Blue_Auto_1.LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    public int last_action = 0;
    public enum path_pos {
        Start,
        FireStart,
        A1,
        A2,
        A3,
        A4,
        A4Fire,
        A5,
        A6,
        A7,
        A8,
        A9,
        end,
    }
    public Blue_Auto_1.path_pos current_path = Blue_Auto_1.path_pos.Start;
    public boolean launched = false;
    public Limelight3A limelight;
    public int obelisk = 21;
    public int launch_counter = 0;
    public long last_fire_time = 0;
    public limelight_thinker limelight_thinker;
    public boolean has_diverted = false;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        telemetry.addLine("I'm Blue Bro");
        leftLaunchState = Blue_Auto_1.LaunchState.IDLE;
        rightLaunchState = Blue_Auto_1.LaunchState.IDLE;
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
        follower = RuntVisualConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }
    public int fire = 0;
    public long fire_pause = 7000;
    public boolean obelisk_changed = false;
    public long divert_time = 0;
    public long divert_wait = 1000;

    @Override
    public void loop() {
        switch (current_path) {
            case Start:
                if (last_action == 0) {
                    
                }
                break;
        }
    }
}

class Blue_Auto_1 extends OpMode{
    //**PRELOAD MUST BE PURPLE ON RIGHT ONE GREEN ON LEFT**
    final double FEED_TIME_SECONDS = 1.45; //The feeder servos run this long when a shot is requested.
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
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


    enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    enum DiverterDirection {
        LEFT,
        RIGHT;
    }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    enum IntakeState {
        ON,
        OFF;
    }

    private IntakeState intakeState = IntakeState.OFF;

    enum LauncherDistance {
        CLOSE,
        FAR;
    }
    private Follower follower;
    public static Pose startingPose = new Pose(68,1,90); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    public int last_action = 0;
    public enum path_pos {
        Start,
        FireStart,
        A1,
        A2,
        A3,
        A4,
        A4Fire,
        A5,
        A6,
        A7,
        A8,
        A9,
        end,
    }
    public path_pos current_path = path_pos.Start;
    public boolean launched = false;
    public Limelight3A limelight;
    public int obelisk = 21;
    public int launch_counter = 0;
    public long last_fire_time = 0;
    public limelight_thinker limelight_thinker;
    public boolean has_diverted = false;
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }


        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(Drawing.ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * Drawing.ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }
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
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }
    public int fire = 0;
    public long fire_pause = 7000;
    public boolean obelisk_changed = false;
    public long divert_time = 0;
    public long divert_wait = 1000;
    Blue_A1_Paths paths;
    @Override
    public void start() {

        intakeState = IntakeState.ON;
        paths = new Blue_A1_Paths(follower);
        diverterDirection = DiverterDirection.LEFT;
        diverter.setPosition(LEFT_POSITION);
        intake.setPower(1);
    }

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    @Override
    public void loop() {
        follower.update();
        drawRobot(follower.getPose(),robotLook);
        telemetry.update();
        telemetry.addData("Current ", current_path.name());
        telemetry.addData("Fire condition ", System.currentTimeMillis() + fire_pause > last_fire_time);
        telemetry.addData("Divert condition ", System.currentTimeMillis() + divert_wait > divert_time);
        telemetry.addData("Divert Position ", diverterDirection.name());
        telemetry.addData("Intake State ", intakeState.name());
        telemetry.addData("Follower Busy? ", follower.isBusy());
        telemetry.addData("Obelisk ", obelisk);
        telemetry.addData("Obelisk Changed? ", obelisk_changed);
        if (!follower.isBusy()) {

            switch (current_path) {
                case Start:
                    if (last_action == 0) {
                        follower.followPath(paths.Path1);
                        last_action = 1;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.FireStart;
                        }
                    }
                    break;
                case FireStart:
                    telemetry.addLine("Running Firestart");
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
                    if (fire == 0) {
                        if (obelisk == 21) {
                            launchLeft(true);
                            launchRight(true);
                            launchRight(true);
                        }
                        if (obelisk == 22) {
                            launchRight(true);
                            launchLeft(true);
                            launchRight(true);
                        }
                        if (obelisk == 23) {
                            launchRight(true);
                            launchRight(true);
                            launchLeft(true);
                        }
                        fire = 1;
                        last_fire_time = System.currentTimeMillis() ;
                        break;
                    } else {
                        if (System.currentTimeMillis() > last_fire_time + fire_pause) {
                            fire = 1;
                            last_fire_time = System.currentTimeMillis();
                            current_path = path_pos.A1;
                        }
                        break;
                    }
                case A1:
                    fire = 0;
                    if (last_action == 1) {
                        follower.followPath(paths.Path2);
                        last_action = 2;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.A2;
                        }
                    }
                    break;
                case A2:
                    if (!has_diverted) {
                        //divert_right
                        has_diverted = true;
                        divert_time = System.currentTimeMillis();
                        diverterDirection = DiverterDirection.RIGHT;
                        diverter.setPosition(RIGHT_POSITION);
                    }
                    if (divert_time + divert_wait <= System.currentTimeMillis()) {
                        if (last_action == 2) {
                            follower.followPath(paths.Path3);
                            last_action = 3;
                        } else {
                            if (follower.atParametricEnd()) {
                                current_path = path_pos.A3;
                            }
                        }
                    }
                    break;
                case A3:
                    has_diverted = false;
                    if (last_action == 3) {
                        follower.followPath(paths.Path4);
                        last_action = 4;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.A4;
                        }
                    }
                    break;
                case A4:
                    if (last_action == 4) {
                        follower.followPath(paths.Path5);
                        last_action = 5;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.A5;
                        }
                    }
                    break;
                case A4Fire:
                    telemetry.addLine("Running Firestart");
                    if (fire == 0) {
                        if (obelisk == 21) {
                            launchLeft(true);
                            launchRight(true);
                            launchRight(true);
                        }
                        if (obelisk == 22) {
                            launchRight(true);
                            launchLeft(true);
                            launchRight(true);
                        }
                        if (obelisk == 23) {
                            launchRight(true);
                            launchRight(true);
                            launchLeft(true);
                        }
                        fire = 1;
                        last_fire_time = System.currentTimeMillis() ;
                        break;
                    } else {
                        if (System.currentTimeMillis() > last_fire_time + fire_pause) {
                            fire = 1;
                            last_fire_time = System.currentTimeMillis();
                            current_path = path_pos.A1;
                        }
                        break;
                    }
                case A5:
                    fire = 0;
                    if (last_action == 5) {
                        follower.followPath(paths.Path6);
                        last_action = 6;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.A6;
                        }
                    }
                    break;
                case A6:
                    if (last_action == 6) {
                        follower.followPath(paths.Path7);
                        last_action = 7;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.A7;
                        }
                    }
                    break;
                case A7:
                    if (!has_diverted) {
                        //divert left
                        has_diverted = true;
                        divert_time = System.currentTimeMillis();
                        diverterDirection = DiverterDirection.LEFT;
                        diverter.setPosition(LEFT_POSITION);
                    }
                    if (divert_time + divert_wait <= System.currentTimeMillis()) {
                        if (last_action == 2) {
                            follower.followPath(paths.Path8);
                            last_action = 3;
                        } else {
                            if (follower.atParametricEnd()) {
                                current_path = path_pos.A8;
                            }
                        }
                    }
                    break;
                case A8:
                    has_diverted = false;
                    if (last_action == 3) {
                        follower.followPath(paths.Path9);
                        last_action = 4;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.end;
                        }
                    }
                    break;
                case end:
                    //purple left, green purple right
                    if (fire == 0) {
                        if (obelisk == 21) {
                            launchRight(true);
                            launchLeft(true);
                            launchRight(true);
                        }
                        if (obelisk == 22) {
                            launchLeft(true);
                            launchRight(true);
                            launchLeft(true);
                        }
                        if (obelisk == 23) {
                            launchRight(true);
                            launchRight(true);
                            launchLeft(true);
                        }
                        fire = 1;
                    }
                    break;
            }
        }
    }

    public class Blue_A1_Paths {
        int translational_offset = 0;
        int rotational_offset = 0;
        private Follower follower;
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Blue_A1_Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(68.000, 1.000), new Pose(62.355, 14.579))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(62.355, 14.579), new Pose(52.486, 36.336))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.486, 36.336), new Pose(36.561, 35.664))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.561, 35.664), new Pose(18.859, 34.438))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.859, 34.438), new Pose(62.355, 13.579))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62.355, 13.579),
                                    new Pose(28.513, 21.640),
                                    new Pose(41.535, 61.379)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.535, 61.379), new Pose(24.023, 60.930))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.023, 60.930), new Pose(12.561, 60.112))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(12.561, 60.112),
                                    new Pose(20.667, 94.600),
                                    new Pose(50.019, 112.374),
                                    new Pose(51.081, 108.882)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
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

class Red_Auto_1 extends OpMode{
    //**PRELOAD MUST BE PURPLE ON RIGHT ONE GREEN ON LEFT**
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

    public int last_action = 0;
    public enum path_pos {
        Start,
        FireStart,
        A1,
        A2,
        A3,
        A4,
        A4Fire,
        A5,
        A6,
        A7,
        A8,
        A9,
        end,
    }
    public path_pos current_path = path_pos.Start;
    public boolean launched = false;
    public Limelight3A limelight;
    public int obelisk = 0;
    public int launch_counter = 0;
    public long last_fire_time = 0;
    public limelight_thinker limelight_thinker;
    public boolean has_diverted = false;

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
        follower = RuntVisualConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }
    public int fire = 0;
    public long fire_pause = 2000;
    public long divert_time = 0;
    public long divert_wait = 1000;
    Blue_A1_Paths paths;
    @Override
    public void start() {

        intakeState = IntakeState.ON;
        paths = new Blue_A1_Paths(follower);
        diverterDirection = DiverterDirection.LEFT;
        diverter.setPosition(LEFT_POSITION);
        intake.setPower(1);
    }


    @Override
    public void loop() {
        follower.update();
        telemetry.update();
        if (!follower.isBusy()) {
            switch (current_path) {
                case Start:
                    if (last_action == 0) {
                        follower.followPath(paths.Path1);
                        last_action = 1;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.FireStart;
                        }
                    }
                    break;
                case FireStart:
                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid()) {
                        double tx = result.getTx();
                        double ty = result.getTy();
                        double ta = result.getTa();
                        for (LLResultTypes.FiducialResult april : result.getFiducialResults()) {
                            if (april.getFiducialId() != 20 || april.getFiducialId() != 24) {
                                obelisk = april.getFiducialId();
                            }
                        }
                    }
                    if (fire == 0) {
                        if (obelisk == 21) {
                            launchLeft(true);
                            launchRight(true);
                            launchRight(true);
                        }
                        if (obelisk == 22) {
                            launchRight(true);
                            launchLeft(true);
                            launchRight(true);
                        }
                        if (obelisk == 23) {
                            launchRight(true);
                            launchRight(true);
                            launchLeft(true);
                        }
                        fire = 1;
                    } else {
                        if (System.currentTimeMillis() + fire_pause >= last_fire_time) {
                            fire = 1;
                            last_fire_time = System.currentTimeMillis();
                            current_path = path_pos.A1;
                        }
                    }
                    break;
                case A1:
                    fire = 0;
                    if (last_action == 1) {
                        follower.followPath(paths.Path2);
                        last_action = 2;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.A2;
                        }
                    }
                    break;
                case A2:
                    if (!has_diverted) {
                        //divert_right
                        has_diverted = true;
                        divert_time = System.currentTimeMillis();
                        diverterDirection = DiverterDirection.RIGHT;
                        diverter.setPosition(RIGHT_POSITION);
                    }
                    if (divert_time + divert_wait <= System.currentTimeMillis()) {
                        if (last_action == 2) {
                            follower.followPath(paths.Path3);
                            last_action = 3;
                        } else {
                            if (follower.atParametricEnd()) {
                                current_path = path_pos.A3;
                            }
                        }
                    }
                    break;
                case A3:
                    has_diverted = false;
                    if (last_action == 3) {
                        follower.followPath(paths.Path4);
                        last_action = 4;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.A4;
                        }
                    }
                case A4:
                    if (last_action == 4) {
                        follower.followPath(paths.Path5);
                        last_action = 5;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.A5;
                        }
                    }
                    break;
                case A4Fire:
                    if (fire == 0) {
                        if (obelisk == 21) {
                            launchLeft(true);
                            launchRight(true);
                            launchRight(true);
                        }
                        if (obelisk == 22) {
                            launchRight(true);
                            launchLeft(true);
                            launchRight(true);
                        }
                        if (obelisk == 23) {
                            launchRight(true);
                            launchRight(true);
                            launchLeft(true);
                        }
                        fire = 1;
                    } else {
                        if (System.currentTimeMillis() + fire_pause >= last_fire_time) {
                            fire = 1;
                            last_fire_time = System.currentTimeMillis();
                            current_path = path_pos.A5;
                        }
                    }
                    break;
                case A5:
                    fire = 0;
                    if (last_action == 5) {
                        follower.followPath(paths.Path6);
                        last_action = 6;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.A6;
                        }
                    }
                    break;
                case A6:
                    if (last_action == 6) {
                        follower.followPath(paths.Path7);
                        last_action = 7;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.A7;
                        }
                    }
                    break;
                case A7:
                    if (!has_diverted) {
                        //divert left
                        has_diverted = true;
                        divert_time = System.currentTimeMillis();
                        diverterDirection = DiverterDirection.LEFT;
                        diverter.setPosition(LEFT_POSITION);
                    }
                    if (divert_time + divert_wait <= System.currentTimeMillis()) {
                        if (last_action == 2) {
                            follower.followPath(paths.Path8);
                            last_action = 3;
                        } else {
                            if (follower.atParametricEnd()) {
                                current_path = path_pos.A8;
                            }
                        }
                    }
                    break;
                case A8:
                    has_diverted = false;
                    if (last_action == 3) {
                        follower.followPath(paths.Path9);
                        last_action = 4;
                    } else {
                        if (follower.atParametricEnd()) {
                            current_path = path_pos.end;
                        }
                    }
                    break;
                case end:
                    //purple left, green purple right
                    if (fire == 0) {
                        if (obelisk == 21) {
                            launchRight(true);
                            launchLeft(true);
                            launchRight(true);
                        }
                        if (obelisk == 22) {
                            launchLeft(true);
                            launchRight(true);
                            launchLeft(true);
                        }
                        if (obelisk == 23) {
                            launchRight(true);
                            launchRight(true);
                            launchLeft(true);
                        }
                        fire = 1;
                    }
                    break;
            }
        }
    }

    public class Blue_A1_Paths {
        int translational_offset = 4;
        int rotational_offset = 20;
        private Follower follower;
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Blue_A1_Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000+translational_offset, 8.000).mirror(), new Pose(62.355+translational_offset, 14.579).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90+rotational_offset), Math.toRadians(66+rotational_offset))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(62.355+translational_offset, 14.579).mirror(), new Pose(52.486+translational_offset, 36.336).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(66+rotational_offset), Math.toRadians(0+rotational_offset))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.486+translational_offset, 36.336).mirror(), new Pose(36.561+translational_offset, 35.664).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0+rotational_offset))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.561+translational_offset, 35.664).mirror(), new Pose(12.785+translational_offset, 34.991).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0+rotational_offset))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.785+translational_offset, 34.991).mirror(), new Pose(62.355+translational_offset, 14.579).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0+rotational_offset), Math.toRadians(66+rotational_offset))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(62.355+translational_offset, 14.579).mirror(), new Pose(53.832+translational_offset, 60.112).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(114+rotational_offset), Math.toRadians(0+rotational_offset))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.832+translational_offset, 60.112).mirror(), new Pose(39.028+translational_offset, 60.112).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0+rotational_offset))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.028+translational_offset, 60.112).mirror(), new Pose(12.561+translational_offset, 60.336).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0+rotational_offset))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(12.561+translational_offset, 60.336),
                                    new Pose(11.664+translational_offset, 78.280),
                                    new Pose(93.757+translational_offset, 85.458),
                                    new Pose(50.019+translational_offset, 112.374)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0+rotational_offset), Math.toRadians(66+rotational_offset))
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
class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
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
        Start,
        A1,
        A2,
        A3,
        A4,
        end,
    }
    public path_pos current_path = path_pos.Start;
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
        List<Detection> detections = limelight_thinker.search();
        follower.update();
        for (Detection detect :detections) {
        if (detect.april == 20) {
            if (detect.flat_position.getX() <= 5) {
                telemetry.addData("X", detect.flat_position.getX());
                follower.turn(detect.flat_position.getX() ,false);
            } else if (detect.flat_position.getX() >= -5 || detect.flat_position.getX() <= -3) {
                telemetry.addData("X", detect.flat_position.getX());
                follower.turn(detect.flat_position.getX() ,true);
            }

        }

    }}
}
class Testing_Blue_A1 extends OpMode{
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

    public boolean has_moved = false;
    public enum path_pos {
        Start,
        A1,
        A2,
        A3,
        A4,
        end,
    }
    public path_pos current_path = path_pos.Start;
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
        follower.followPath(new Blue_A1_Paths(follower).Path1);
    }
    @Override
    public void loop() {
        follower.update();
        telemetry.update();

        telemetry.addData("Path", current_path.name());
        Drawing.drawRobot(follower.getPose());
        List<Detection> detections = limelight_thinker.search();
        for (Detection detect :detections) {
            if (detect.april != 0 || detect.april != 20) {
                telemetry.addData("Selected Obelisk", detect.april);
                obelisk = detect.april;
            }
        }
        if (!follower.isBusy()) {
            switch (current_path) {
                case Start:
                    launcherDistance = LauncherDistance.FAR;
                    launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                    if (has_moved){
                        current_path = path_pos.A1;
                        break;
                    }
                    follower.followPath(new Blue_A1_Paths(follower).Path1);
                    has_moved = true;
                    break;
                case A1:
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    if (follower.atParametricEnd() && !has_moved) {
                        current_path = path_pos.A2;
                        break;
                    }
                    follower.followPath(new Blue_A1_Paths(follower).Path2);
                    has_moved = false;
                    break;
                case A2:
                    if (follower.atParametricEnd() && has_moved) {
                        current_path = path_pos.A3;
                    }
                    follower.followPath(new Blue_A1_Paths(follower).Path3);
                    has_moved = true;
                    break;
                case A3:
                    if (follower.atParametricEnd() && !has_moved) {
                        current_path = path_pos.A4;
                        break;
                    }
                    follower.followPath(new Blue_A1_Paths(follower).Path4);
                    has_moved = false;
                case A4:
                    //logic for shooting correct
                    //two purple left one green right
                    follower.turnTo(Math.toRadians(134));

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
        }
//        if (!follower.isBusy()) {
//            switch (current_path) {
//                case Start:
//                    follower.followPath(new Blue_A1_Paths(follower).Path1);
//                case A1:
//                    launcherDistance = LauncherDistance.FAR;
//                    launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
//                    launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
//                    follower.followPath(new Blue_A1_Paths(follower).Path2);
//                    break;
//                case A2:
//                    follower.followPath(new Blue_A1_Paths(follower).Path3);
//                    break;
//                case A3:
//                    follower.followPath(new Blue_A1_Paths(follower).Path4);
//                    break;
//            }
//        }
//        if (follower.atParametricEnd() && !follower.isBusy()) {
//            switch (current_path) {
//                case Start:
//                    follower.followPath(new Blue_A1_Paths(follower).Path1);
//                    current_path = path_pos.A1;
//                case A1:
//                    telemetry.addData("Current", current_path.name());
//                    current_path = path_pos.A2;
//                case A2:
//
//                    current_path = path_pos.A3;
//                    break;
//                case A3:
//                    diverterDirection = DiverterDirection.LEFT;
//                    diverter.setPosition(LEFT_POSITION);
//                    current_path = path_pos.A4;
//                    break;
//                case A4:
//                    current_path = path_pos.end;
//                    break;
//                case end:
//                    //logic for shooting correct
//                    //two purple left one green right
//                    if (obelisk == 21) {
//                        //green purple purple
//                        launchRight(true);
//                        if (rightLaunchState == LaunchState.IDLE && !launched) {
//                            launched = true;
//                            launchLeft(true);
//                        }
//                        if (rightLaunchState == LaunchState.IDLE && launched) {
//                            launchLeft(true);
//
//                        }
//
//                    }
//                    if (obelisk == 22) {
//                        if (leftLaunchState == LaunchState.IDLE && !launched) {
//                            launchLeft(true);
//                            launched = true;
//                        }
//                        if (launched && leftLaunchState == LaunchState.IDLE) {
//                            launchRight(true);
//                            launched = false;
//                        }
//                    }
//                    if (obelisk == 23) {
//                        if (leftLaunchState == LaunchState.IDLE && !launched) {
//                            launchLeft(true);
//                            launch_counter = 1;
//                        } else if (!launched && leftLaunchState == LaunchState.IDLE && launch_counter == 1) {
//                            launch_counter = 2;
//                            launchRight(true);
//                            launched = true;
//                        }
//
//                    }
//                    break;
        telemetry.addLine("On Path" + current_path.name());
            }
    public void Launch(){

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
                            new BezierLine(new Pose(67.354+4, 0.312), new Pose(56.577+4, 28.151))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(200))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.577+4, 28.151), new Pose(36.147+4, 29.947))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(200))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.147+4, 29.947), new Pose(15.267+4, 29.723))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(200))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.267+4, 29.723), new Pose(65.333+4, 10.639))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(134))
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