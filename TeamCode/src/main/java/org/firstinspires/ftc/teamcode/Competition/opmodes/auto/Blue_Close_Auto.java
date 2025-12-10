package org.firstinspires.ftc.teamcode.Competition.opmodes.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher.delagate_shots;
import static org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher.launchLeft;
import static org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher.launchRight;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.PathCallback;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Competition.opmodes.Teleop.Competition_Teleop;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;


@Configurable
@Autonomous(name = "Blue Close Auto")
public class Blue_Close_Auto extends OpMode {
    //**PRELOAD MUST BE PURPLE ON RIGHT ONE GREEN ON LEFT**
    //TODO Fix Shooting, Finish Bottom parts of functionality, as listed with other TODOS
    public int loops = 0;
    public static double left_feeder_block = 0.25;
    public static double right_feeder_block = 0.25;
    public static double right_feeder_up = 0;
    public static double left_feeder_up = 1;
    public static double shoot_degree = 160;
    public static Pose shoot_pos = new Pose(81.140,115.832,Math.toRadians(shoot_degree));

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
    ElapsedTime rightFeederTimer = new ElapsedTime();
    final double FEED_TIME_SECONDS = 1.45; //The feeder servos run this long when a shot is requested.
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    public static double LAUNCHER_CLOSE_TARGET_VELOCITY = 1150; //in ticks/second for the close goal.
    public static double LAUNCHER_CLOSE_MIN_VELOCITY = 1100; //minimum required to start a shot for close goal.
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
    @Override
    public void init() {
        leftLaunchState = LaunchState.IDLE;
        rightLaunchState =LaunchState.IDLE;

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
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
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

        //*‼️QUICK RULE OF THUMB‼️* Make sure to use the correct constants!
        //(❁´◡`❁)
        follower = RuntOdometryWheelsConstants.createFollower(hardwareMap);
        //*‼️QUICK RULE OF THUMB‼️* Make sure to set starting pose with the following format, Double Double Radians
        follower.setStartingPose(new Pose(21.981308411214954,124.93457943925235,Math.toRadians(135)));
        follower.update();
        Ze_Paths = new Da_Paths(follower);
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        telemetry.addData("This is the concept for the Close Auto for 20525 with config 'Silly Robot'", "This DOES move (off the table)");
    }
    public enum paths {
        Path1,
        Path2,
        Path2FIRE,
        Path3,
        Path4,
        Path5,
        Path6,
        Path6Fire,
        Path7,
        Path8,
        Path9,
        Path10,
        end
    }
    @Override
    public void start() {
        leftFeeder.setPosition(left_feeder_block);
        rightFeeder.setPosition(right_feeder_block);
    }
    // Initialize Da_Paths first to avoid having to remake it every loop
    public Da_Paths Ze_Paths;
    // We need to set initial state, change it during runtime or before to set states
    public paths currentpath = paths.Path1;
    // Just so we know if it reads
    public boolean obelisk_changed;
    public int obelisk = 21;
    // This is a long, 6000 = 6 seconds
    public static long fire_wait = 6000; //Originally 6000;
    // To be changed during runtime
    public long last_fire_time = 0;
    public int fire = 0;
    public double first_wait = 0.55;
    public double second_wait = 0.65;
    public double third_wait =  0.80;
    @Override
    public void loop() {
        // We log both Launcher Velocities
        telemetry.addData("Right Launcher", rightLauncher.getVelocity());
        telemetry.addData("Left Launcher", leftLauncher.getVelocity());
        // We ALWAYS keep motors spinned up along with Intake
        leftLauncher.setVelocity(launcherTarget);
        rightLauncher.setVelocity(launcherTarget);
        intake.setPower(1);
        follower.update();
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Pose", follower.getPose());
        // Only index during the correct states
        // These States being Path2Fire, Path6Fire, and End
        // TODO Make it actually aim
        if ((currentpath != paths.Path2FIRE && currentpath != paths.Path6Fire && currentpath != paths.end)) {
            telemetry.addData("Conditional", (currentpath != paths.Path2FIRE || currentpath != paths.Path6Fire || currentpath != paths.end));
            feeder.setPower(0);
        }
        // ️️ ‼️*Quick Rule of Thumb*‼️ Only follow new path at parametric end, and do not find parametric end at start of loop
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
            case Path2:
                // Keep it diverted right then follow path 2
                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path2);
                    currentpath = paths.Path2FIRE;
                }
                break;
            case Path2FIRE:
                // Keep it diverted!
                // Then fire every around 10-20% of the time after 0.35
                //purple on right green on left
                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
                if (follower.atParametricEnd()) {
                if (fire == 0) {
                    last_fire_time = System.currentTimeMillis();
                    fire = 1;
                }
                if (last_fire_time + (fire_wait - (5000)) <= System.currentTimeMillis()) {
                    delagate_shots(fire_wait,last_fire_time, Launcher.insides.GPP,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,obelisk, Launcher.LauncherDistance.FAR);
//                    switch (obelisk) {
//                        case 21:
//                            //Left Right Right
//                            // aka Green Purple Purple
//                            if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
//                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
//                            }
//                            if (last_fire_time + (fire_wait * second_wait /*originally 0.6 */) <= System.currentTimeMillis()) {
//                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
//                            }
//                            if (last_fire_time + (fire_wait * third_wait /* Originally 0.7*/) <= System.currentTimeMillis()) {
//
//                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
//                            }
//                            break;
//                        case 22:
//                            // Right Left Right
//                            // aka Purple Green Purple
//                            if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
//                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
//                            }
//                            if (last_fire_time + (fire_wait * second_wait) <= System.currentTimeMillis()) {
//                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
//                            }
//                            if (last_fire_time + (fire_wait * third_wait) <= System.currentTimeMillis()) {
//
//                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
//                            }
//                            break;
//                        case 23:
//                            // Right Right Left
//                            // aka Purple Purple Green 😎
//                            if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
//                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
//                            }
//                            if (last_fire_time + (fire_wait * second_wait) <= System.currentTimeMillis()) {
//                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
//                            }
//                            if (last_fire_time + (fire_wait * third_wait) <= System.currentTimeMillis()) {
//
//                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
//                            }
//                            break;
//                    }
                }

                if (delagate_shots(fire_wait,last_fire_time, Launcher.insides.GPP,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,obelisk, Launcher.LauncherDistance.FAR)) {
                    // Then go to next path
                    currentpath = paths.Path3;
                }

                }
                break;
            case Path3:
                //Keep Diverter Right!
                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path3);
                    currentpath = paths.end;
                }
                break;
            case Path4:
                //Pick up First ball go slow slow (0.5 Power)
                if (follower.atParametricEnd()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(Ze_Paths.Path4);
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    currentpath = paths.Path4;

                }
            case Path5:
                // Flip Diverter to keep Green on Left, Purple on Right
                diverterDirection = DiverterDirection.LEFT;
                diverter.setPosition(LEFT_POSITION);
                if (follower.atParametricEnd()) {
                    // Follow Next path
                    follower.followPath(Ze_Paths.Path5);
                    currentpath = paths.Path4;
                }
            case Path6:
                // Go speedy and then go to shoot
                if (follower.atParametricEnd()) {
                    follower.setMaxPower(1);
                    follower.followPath(Ze_Paths.Path6);
                    currentpath = paths.Path6Fire;
                }
            case Path6Fire:
                //wait 3 seconds
                if (follower.atParametricEnd()) {
                if (fire == 1) {
                    last_fire_time = System.currentTimeMillis();
                    fire = 2;
                }
                if (last_fire_time + (fire_wait - (5000)) <= System.currentTimeMillis()) {
                        switch (obelisk) {
                            case 21:
                                //Left Right Right
                                // aka Green Purple Purple
                                if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
                                    launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
                                }
                                if (last_fire_time + (fire_wait * second_wait) <= System.currentTimeMillis()) {
                                    launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
                                }
                                if (last_fire_time + (fire_wait * third_wait) <= System.currentTimeMillis()) {

                                    launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
                                }
                                break;
                            case 22:
                                // Right Left Right
                                // aka Purple Green Purple
                                if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
                                    launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
                                }
                                if (last_fire_time + (fire_wait * second_wait) <= System.currentTimeMillis()) {
                                    launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
                                }
                                if (last_fire_time + (fire_wait * third_wait) <= System.currentTimeMillis()) {

                                    launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
                                }
                                break;
                            case 23:
                                // Right Right Left
                                // aka Purple Purple Green 😎
                                if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
                                    launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
                                }
                                if (last_fire_time + (fire_wait * second_wait) <= System.currentTimeMillis()) {
                                    launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
                                }
                                if (last_fire_time + (fire_wait * third_wait) <= System.currentTimeMillis()) {

                                    launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,Launcher.LauncherDistance.CLOSE);
                                }
                                break;
                        }
                    }
                if (last_fire_time + fire_wait <= System.currentTimeMillis()) {
                    //Follow Next Path
                    currentpath = paths.Path7;
                }
                }
                break;
            case Path7:
                // Head Backdown
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path7);
                    currentpath = paths.Path8;
                }
            case Path8:
                // Slow your roll bud
                if (follower.atParametricEnd()) {
                    follower.setMaxPower(0.5);
                        follower.followPath(Ze_Paths.Path8);
                        currentpath = paths.Path9;
                }
            case Path9:
                // TODO swap diverter
                // Intake
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path9);
                    currentpath = paths.Path10;
                }
            case Path10:
                // Fly back up
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path10);
                    follower.setMaxPower(1);
                    currentpath = paths.end;
                }
            case end:
                //wait 3 seconds
                // TODO add shooting
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
    


    private static class Da_Paths {
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
                            new BezierLine(new Pose(21.981, 124.935), new Pose(37.009, 105.421))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(37.009, 105.421), shoot_pos)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(shoot_degree))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shoot_pos, new Pose(42.841, 79.626))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(shoot_degree), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.841, 79.626), new Pose(32.299, 79.402))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(32.299, 79.402), new Pose(14.131, 79.178))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.131, 79.178), shoot_pos)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shoot_pos, new Pose(42.168, 59.439))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.168, 59.439), new Pose(32.299, 59.215))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(32.299, 59.215), new Pose(12.561, 58.991))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.561, 58.991), new Pose(50.916, 126.729))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
        }

    }


