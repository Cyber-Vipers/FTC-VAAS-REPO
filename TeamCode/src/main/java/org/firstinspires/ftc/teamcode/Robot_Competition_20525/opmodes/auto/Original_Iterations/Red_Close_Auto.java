package org.firstinspires.ftc.teamcode.Robot_Competition_20525.opmodes.auto.Original_Iterations;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Util.Robot_Composition;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Sphere_Operator;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher.launchLeft;
import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher.launchRight;


public class Red_Close_Auto extends OpMode {
    //**PRELOAD MUST BE PURPLE ON RIGHT ONE GREEN ON LEFT**
    //TODO Fix Shooting, Finish Bottom parts of functionality, as listed with other TODOS
    public int loops = 0;
    public static double left_feeder_block = 0.25;
    public static double right_feeder_block = 0.25;
    public static double right_feeder_up = 0;
    public static double left_feeder_up = 1;
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState = LaunchState.IDLE;
    private LaunchState rightLaunchState = LaunchState.IDLE;
    public DcMotor feeder;
    public double first_wait = 0.55;
    public double second_wait = 0.65;
    public double third_wait =  0.80;
    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();
    final double FEED_TIME_SECONDS = 1.45; //The feeder servos run this long when a shot is requested.
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    Rev2mDistanceSensor color1;
    Rev2mDistanceSensor color2;

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
    Robot_Composition robotComposition;

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
    public Servo limelight_servo;
    private Servo diverter = null;
    private Follower follower;
    public Limelight3A limelight;
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
        color2 = hardwareMap.get(Rev2mDistanceSensor.class, "color2");

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
        follower.setStartingPose(new Pose(127.52329576522467,125.1409210488048,Math.toRadians(35)));
        follower.update();
        Ze_Paths = new Da_Paths(follower);
        robotComposition = new Robot_Composition(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, leftLauncher, rightLauncher, intake, feeder, leftFeeder, rightFeeder,color1,color2,follower,telemetry,diverter,limelight_servo,limelight);
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
    // Initialize Da_Paths first to avoid having to remake it every loop
    public Da_Paths Ze_Paths;
    // We need to set initial state, change it during runtime or before to set states
    public paths currentpath = paths.Path1;
    // Just so we know if it reads
    public boolean obelisk_changed;
    public int obelisk = 21;
    // This is a long, 6000 = 6 seconds
    public long fire_wait = 6000;
    // To be changed during runtime
    public long last_fire_time = 0;
    public int fire = 0;
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
                        switch (obelisk) {
                            case 21:
                                //Left Right Right
                                // aka Green Purple Purple
                                if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
                                    launchLeft(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * second_wait /*originally 0.6 */) <= System.currentTimeMillis()) {
                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * third_wait /* Originally 0.7*/) <= System.currentTimeMillis()) {

                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                break;
                            case 22:
                                // Right Left Right
                                // aka Purple Green Purple
                                if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * second_wait) <= System.currentTimeMillis()) {
                                    launchLeft(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * third_wait) <= System.currentTimeMillis()) {

                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                break;
                            case 23:
                                // Right Right Left
                                // aka Purple Purple Green 😎
                                if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * second_wait) <= System.currentTimeMillis()) {
                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * third_wait) <= System.currentTimeMillis()) {

                                    launchLeft(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                break;
                        }
                    }

                    if (last_fire_time + fire_wait <= System.currentTimeMillis()) {
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
                    currentpath = paths.Path4;
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
                                    launchLeft(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * second_wait) <= System.currentTimeMillis()) {
                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * third_wait) <= System.currentTimeMillis()) {

                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                break;
                            case 22:
                                // Right Left Right
                                // aka Purple Green Purple
                                if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * second_wait) <= System.currentTimeMillis()) {
                                    launchLeft(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * third_wait) <= System.currentTimeMillis()) {

                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                break;
                            case 23:
                                // Right Right Left
                                // aka Purple Purple Green 😎
                                if (last_fire_time + (fire_wait * first_wait)  <= System.currentTimeMillis()) {
                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * second_wait) <= System.currentTimeMillis()) {
                                    launchRight(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
                                }
                                if (last_fire_time + (fire_wait * third_wait) <= System.currentTimeMillis()) {

                                    launchLeft(true,robotComposition,Launcher.LauncherDistance.CLOSE,0);
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
}


