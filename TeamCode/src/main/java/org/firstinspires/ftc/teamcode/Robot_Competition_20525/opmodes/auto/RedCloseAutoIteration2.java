
package org.firstinspires.ftc.teamcode.Robot_Competition_20525.opmodes.auto;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Sphere_Operator.handle_divert;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;


import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Util.Robot_Composition;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;


@Autonomous(name = "Red Close Auto", group = "Autonomous")
@Configurable // Panels
public class RedCloseAutoIteration2 extends OpMode {
    public static double shoot_degree = Math.toRadians(38);
    public static double intake_degree = Math.toRadians(0);
    public static Pose shoot_pos = new Pose(86.579,106.542,Math.toRadians(shoot_degree));

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
    private Paths paths; // Paths defined in the Paths class

    public enum States {
        Initial,
        Shoot,
        PrepareIntake,
        ApproachIntake1,
        Intake1,
        Shoot2,
        PrepareIntake2,
        ApproachIntake2,
        Intake2,
        Shoot3,
        Leave
    }
    public States current_state = States.Initial;

    public Pose startingPose = new Pose(122.019, 124.935, Math.toRadians(45));
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

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        robotComposition = new Robot_Composition(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, leftLauncher, rightLauncher, intake, feeder, leftFeeder, rightFeeder,color1,color2,follower,telemetry,diverter,limelight_servo,limelight);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        panelsField.update();
        move();
        prevent_false_shots();
        double distance_left = color1.getDistance(DistanceUnit.CM);
        double distance_right = color2.getDistance(DistanceUnit.CM);
        handle_divert(robotComposition,distance_right,distance_left);
        paths = new Paths(follower);
        // Log values to Panels and Driver Station
        telemetry.addData("Current Path", current_state.name());
        panelsTelemetry.debug("Current Path", current_state.name());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        telemetry.update();
        panelsTelemetry.update(telemetry);

    }

    public static long fire_wait_time = 2500;
    public ElapsedTime shoot_timer = new ElapsedTime();
    public ElapsedTime pizza_time = new ElapsedTime();
    public void prevent_false_shots() {
        if (current_state != States.Shoot && current_state != States.Shoot2 && current_state != States.Shoot3) {
            Launcher.shut_gates(robotComposition);
        }
    }
    public ElapsedTime initial_wait = new ElapsedTime();
    public void move() {
        switch (current_state) {
            case Initial:
                if (shoot_timer.milliseconds() > 600) {
                    intake.setPower(1);
                    follower.followPath(paths.Shoot);
                    current_state = States.Shoot;
                    Launcher.spinup(robotComposition, Launcher.LauncherDistance.CLOSE, 0);
                    shoot_timer.reset();
                    shoot_timer.startTime();
                    initial_wait.startTime();
                }

                break;
            case Shoot:
                follower.setMaxPower(1);
                if (follower.atParametricEnd()) {
                    if (shoot_timer.milliseconds() > fire_wait_time + 2000) {
                        follower.followPath(paths.PrepareIntake);
                        current_state =States.PrepareIntake;
                    } else {
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.90) {
                            Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.70) {
                            Launcher.launchLeft(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.60) {
                            Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.10) {
                            Launcher.spam(robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                    }
                }
                break;
            case PrepareIntake:

                if (follower.atParametricEnd()) {
                    follower.followPath(paths.Shoot);
                    current_state = States.ApproachIntake1;
//                    follower.followPath(paths.ApproachIntake1);
//                    current_state = States.ApproachIntake1;
                }
                break;
            case ApproachIntake1:
                follower.setMaxPower(0.3);

                if (follower.atParametricEnd()) {
                    intake.setPower(1);
                    follower.followPath(paths.Intake1);
                    current_state = States.Intake1;
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
                break;
            case Intake1:
                shoot_timer.reset();
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.Shoot2);
                    current_state = States.Shoot2;
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
                break;
            case Shoot2:
                follower.setMaxPower(1);
                if (follower.atParametricEnd()) {
                    if (shoot_timer.milliseconds() > fire_wait_time) {
                        follower.followPath(paths.PrepareIntake2);
                        current_state = States.PrepareIntake2;
                    } else {
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.90) {
                            Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.70) {
                            Launcher.launchLeft(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.60) {
                            Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.10) {
                            Launcher.spam(robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                    }
                }
                break;
            case PrepareIntake2:
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.ApproachIntake2);
                    intake.setPower(1);
                    current_state = States.ApproachIntake2;
                }
                break;
            case ApproachIntake2:
                follower.setMaxPower(0.3);
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.Intake2);
                    current_state = States.Intake2;
                }
                break;
            case Intake2:
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.Shoot3);
                    current_state = States.Shoot3;
                    shoot_timer.reset();
                    shoot_timer.startTime();
                    current_state = States.Shoot3;
                }
                break;
            case Shoot3:
                follower.setMaxPower(1);
                if (follower.atParametricEnd()) {
                    if (shoot_timer.milliseconds() > fire_wait_time) {
                        follower.followPath(paths.Leave);
                        current_state = States.Shoot3;
                    } else {
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.90) {
                            Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.70) {
                            Launcher.launchLeft(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.60) {
                            Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.10) {
                            Launcher.spam(robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                    }
                }
                break;
        }
    }

    public static class Paths {
        public PathChain Shoot;
        public PathChain PrepareIntake;
        public PathChain ApproachIntake1;
        public PathChain Intake1;
        public PathChain Shoot2;
        public PathChain PrepareIntake2;
        public PathChain ApproachIntake2;
        public PathChain Intake2;
        public PathChain Shoot3;
        public PathChain Leave;

            public Paths(Follower follower) {
                Shoot = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(122.019, 124.935),

                                        shoot_pos
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(45), shoot_degree)

                        .build();

                PrepareIntake = follower.pathBuilder().addPath(
                                new BezierLine(
                                        shoot_pos,

                                        new Pose(94.800, 83.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(shoot_degree), Math.toRadians(intake_degree))

                        .build();

                ApproachIntake1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(94.800, 83.000),

                                        new Pose(110.000, 83.000)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(intake_degree))

                        .build();

                Intake1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(110.000, 83.000),

                                        new Pose(134.000, 83.000)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(intake_degree))

                        .build();

                Shoot2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(134.000, 83.000),

                                        shoot_pos
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(intake_degree), Math.toRadians(38))

                        .build();

                PrepareIntake2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        shoot_pos,

                                        new Pose(94.800, 58.500)
                                )
                        ).setLinearHeadingInterpolation(shoot_degree, intake_degree)

                        .build();

                ApproachIntake2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(94.800, 58.500),

                                        new Pose(110.000, 58.500)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(intake_degree))

                        .build();

                Intake2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(110.000, 58.500),

                                        new Pose(134.000, 58.500)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(intake_degree))

                        .build();

                Shoot3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(134.000, 58.500),

                                        shoot_pos
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(intake_degree), Math.toRadians(shoot_degree))

                        .build();

                Leave = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(86.579, 106.542),

                                        new Pose(87.000, 53.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(shoot_degree), Math.toRadians(90))

                        .build();
            }
    }
}






    