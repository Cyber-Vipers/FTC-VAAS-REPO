package org.firstinspires.ftc.teamcode.Competition.opmodes.auto;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.Competition.sensors.limelight_thinker;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntVisualConstants;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


@Autonomous(name = "Simple Auto")
public class Simple_Auto extends OpMode {
    //**PRELOAD MUST BE PURPLE ON RIGHT ONE GREEN ON LEFT**
    public int loops = 0;
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
    private Follower follower;
    @Override
    public void init() {
        telemetry.addLine("I'm Blue Bro");
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
        follower.setStartingPose(new Pose(87.32921482804824,45.87162125949082,90));
        follower.update();
        Ze_Paths = new Super_Paths(follower);
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        telemetry.addData("This is a Simple Auto for testing concepts for 20525 with config 'Silly Robot'", " This DOES move (off the table)");
    }
    public enum paths {
        A1,
        A2,
        A3,
        A4,
        A5
    }
    public Super_Paths Ze_Paths;
    public paths currentpath = paths.A1;
    @Override
    public void loop() {
        follower.update();
        switch (currentpath) {
            case A1:
                if (loops == 0) {
                    follower.followPath(Ze_Paths.Path1);
                    currentpath = paths.A2;
                }
                break;
            case A2:
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path2);
                    currentpath = paths.A3;
                }
                break;
            case A3:
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path3);
                    currentpath = paths.A4;
                }
                break;
            case A4:
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path4);
                    currentpath = paths.A5;
                }
                break;
            case A5:
                if (follower.atParametricEnd()) {
                    follower.followPath(Ze_Paths.Path5);
                    currentpath = paths.A1;
                    loops = loops + 1;
                }
                break;
        }
    }


    public static class Super_Paths {


        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public double baller = -1;
        public PathChain Path4;
        public PathChain Path5;

        public Super_Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.329, 45.872), new Pose(95.364, 53.116).scale(-baller))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.364, 53.116), new Pose(106.955, 58.780).scale(-baller))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(106.955, 58.780), new Pose(108.931, 38.364).scale(-baller))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(108.931, 38.364), new Pose(87.197, 27.827).scale(-baller))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.197, 27.827), new Pose(87.329, 45.608).scale(-baller))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();
        }
    }
}
