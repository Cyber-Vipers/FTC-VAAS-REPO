package org.firstinspires.ftc.teamcode.Robot_Competition_20525.opmodes.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Util.Robot_Composition;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.opmodes.Teleop.Competition_Teleop;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;

@Autonomous(name = "Blue Far Auto", group = "Autonomous")
@Disabled
public class BlueFarAutoIteration2 extends OpMode {




    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();





    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    public Servo limelight_servo;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private DcMotor intake = null;
    private Servo leftFeeder = null;
    private Servo rightFeeder = null;
    private Servo diverter = null;
    public Rev2mDistanceSensor color1;
    public Rev2mDistanceSensor color2;
    public DcMotor feeder;
    public Limelight3A limelight;
    public Robot_Composition robotComposition;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private BlueFarAutoIteration2.Paths paths; // Paths defined in the Paths class

    public enum States {
        Initial,
        Shoot,
        ApproachIntake,
        Intake,
        Shoot2,
        ApproachIntake2,
        IntakeSide,
        Tangent,
        Leave,
        LiorStyle1,
        LiorStyle2,
        Shoot3,
        LiorReturn
    }
    public States current_state = States.Initial;

    public Pose startingPose = new Pose(56, 8, Math.toRadians(90));
    @Override
    public void init() {
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
        color1 = hardwareMap.get(Rev2mDistanceSensor.class,"color2");
        color2 = hardwareMap.get(Rev2mDistanceSensor.class, "color1");
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
        leftFeeder.setDirection(Servo.Direction.REVERSE);
        rightFeeder.setDirection(Servo.Direction.REVERSE);
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

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = RuntOdometryWheelsConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

        paths = new BlueFarAutoIteration2.Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        robotComposition = new Robot_Composition(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, leftLauncher, rightLauncher, intake, feeder, leftFeeder, rightFeeder,color1,color2,follower,telemetry,diverter,limelight_servo,limelight);
        Competition_Teleop.lastauto = Competition_Teleop.LastAutoStates.BlueFar;

    }

    @Override
    public void start() {
        endTimer.reset();
        endTimer.startTime();
    }
    public static Pose EndingPose;
    public static ElapsedTime endTimer = new ElapsedTime();
    public static Pose shoot_pos = new Pose(54.430, 13.570);
    public static double shoot_heading = 106;
    public static double intake_heading = 180;
    public static double down_bad_intake_heading = 270;


    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        panelsField.update();
        prevent_false_shots();
        move();
        EndingPose = follower.getPose();
        Launcher.spinup(robotComposition, Launcher.LauncherDistance.FAR,200);
        // Log values to Panels and Driver Station
        telemetry.addData("Current Shoot Time", shoot_timer.milliseconds());
        telemetry.addData("Shoot Condition", shoot_timer.milliseconds() > fire_wait_time);
        telemetry.addData("Current Path", current_state.name());
        telemetry.addData("LEFT FLYWHEEL VELOCITY", leftLauncher.getVelocity());
        telemetry.addData("RIGHT FLYWHEEL VELOCITY", rightLauncher.getVelocity());

        panelsTelemetry.debug("Current Path", current_state.name());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        telemetry.update();
        panelsTelemetry.update(telemetry);

    }

    public static long fire_wait_time = 4500;
    public ElapsedTime shoot_timer = new ElapsedTime();
    public void prevent_false_shots() {
        if (current_state != States.Shoot && current_state != States.Shoot2) {
            Launcher.shut_gates(robotComposition);
            Launcher.noshoot = true;
        } else {
            Launcher.noshoot = false;
        }
    }

    public void move() {
        if (endTimer.seconds() > 28 && current_state != States.Leave) {
            follower.followPath(paths.Leave);
            current_state = States.Leave;
        }
        switch(current_state){
            case Initial:
                follower.followPath(paths.Shoot);
                current_state = States.Shoot;
                shoot_timer.reset();
                shoot_timer.startTime();
                break;
            case Shoot2:
                if (follower.atParametricEnd()) {
                if (shoot_timer.milliseconds() >= fire_wait_time) {
                    current_state = States.ApproachIntake2;
                    follower.followPath(paths.ApproachIntake2);
                } else {
                    Launcher.launchRight(true,robotComposition, Launcher.LauncherDistance.FAR,0,true,1299);
                    Launcher.launchLeft(true,robotComposition, Launcher.LauncherDistance.FAR,0,true,1299);
                }
                } else {
                        shoot_timer.reset();
                        shoot_timer.startTime();
                }
                break;
            case Shoot:
                if (follower.atParametricEnd()) {
                    if (shoot_timer.milliseconds() >= fire_wait_time) {
                        current_state = States.ApproachIntake;
                        follower.followPath(paths.ApproachIntake);
                    } else {
                        Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.CLOSE, 0, true, 1250);
                        Launcher.launchLeft(true, robotComposition, Launcher.LauncherDistance.CLOSE, 0, true, 1250);
                    }
                } else {
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
                break;
            case ApproachIntake:
                if (follower.atParametricEnd()) {
                    follower.setMaxPower(0.7);
                    current_state = States.Intake;
                    follower.followPath(paths.Intake);

                }
                break;
            case Intake:
                shoot_timer.reset();
                shoot_timer.startTime();
                if (follower.atParametricEnd()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.Shoot2);
                    current_state = States.Shoot2;
                }
                break;
            case Shoot3:
                if (follower.atParametricEnd()) {
                    if (shoot_timer.milliseconds() >= fire_wait_time) {
                        current_state = States.LiorStyle1;
                        follower.followPath(paths.LiorStyle1);
                    } else {
                        Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.CLOSE, 0, true, 1250);
                        Launcher.launchLeft(true, robotComposition, Launcher.LauncherDistance.CLOSE, 0, true, 1250);
                    }
                } else {
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
                break;
            case IntakeSide:
                shoot_timer.reset();
                shoot_timer.startTime();
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.Shoot2);
                    current_state = States.Shoot3;
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
                break;
            case ApproachIntake2:
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.IntakeSide);
                    current_state = States.IntakeSide;
                }
                break;
            case Leave:
                if (follower.getCurrentPathChain() != paths.Leave) {
                    follower.followPath(paths.Leave);
                    current_state = States.Leave;
                }
                break;
            case LiorStyle1:
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.LiorStyle2);
                    current_state = States.LiorStyle2;
                }
                break;
            case LiorStyle2:
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.LiorReturn);
                    current_state = States.LiorReturn;
                }
                break;
            case LiorReturn:
                if (follower.atParametricEnd()) {
                    if (shoot_timer.milliseconds() >= fire_wait_time) {
                        current_state = States.LiorStyle1;
                        follower.followPath(paths.LiorStyle1);
                    } else {
                        Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.CLOSE, 0, true, 1250);
                        Launcher.launchLeft(true, robotComposition, Launcher.LauncherDistance.CLOSE, 0, true, 1250);
                    }
                } else {
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
                break;
        }
    }
    public static double baller = 200;

    public static class Paths {
        public PathChain Shoot;
        public PathChain ApproachIntake;
        public PathChain Intake;
        public PathChain Shoot2;
        public PathChain ApproachIntake2;
        public PathChain IntakeSide;
        public PathChain Tangent;
        public PathChain Leave;
        public PathChain LiorStyle1;
        public PathChain LiorStyle2;
        public PathChain LiorReturn;

        public Paths(Follower follower) {
            Shoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(54.430, 13.570)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(shoot_heading))

                    .build();

            ApproachIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.430, 13.570),

                                    new Pose(50.813, 37.224)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(shoot_heading), Math.toRadians(180))

                    .build();

            Intake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.813, 37.224),

                                    new Pose(18.047, 37.224)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
            Leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shoot_pos,

                                    new Pose(54.346, 44.981)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(shoot_heading), Math.toRadians(90))

                    .build();
            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.047, 37.224),

                                    new Pose(54.430, 13.570)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shoot_heading))

                    .build();

            ApproachIntake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(54.430, 13.570),
                                    new Pose(22.986, 31.682),
                                    new Pose(43.463, 25.159),
                                    new Pose(18.981, 27.327)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(shoot_heading), Math.toRadians(270))

                    .build();

            IntakeSide = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.981, 27.327),

                                    new Pose(19.981, 11.178)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();

            Tangent = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.981, 11.178),

                                    new Pose(54.430, 13.570)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(shoot_heading))

                    .build();

            LiorStyle1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.430, 13.570),

                                    new Pose(19.056, 37.355)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(shoot_heading), Math.toRadians(180))

                    .build();

            LiorStyle2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.056, 37.355),
                                    new Pose(4.958, 37.682),
                                    new Pose(19.654, 23.617)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shoot_heading))

                    .build();

            LiorReturn = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.654, 23.617),

                                    new Pose(54.430, 13.570)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(shoot_heading), Math.toRadians(shoot_heading))

                    .build();
        }
    }





}
