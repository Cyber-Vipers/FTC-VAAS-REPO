package org.firstinspires.ftc.teamcode.Robot_Competition_20525.opmodes.auto;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Util.Robot_Composition;
import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@Autonomous(name = "Red Far Auto", group = "Autonomous")
public class RedFarAutoIteration2 extends OpMode {
    public static double shoot_degree = Math.toRadians(147);
    public static double intake_degree = Math.toRadians(186);
    public static Pose shoot_pos = new Pose(57.42056074766355,106.54205607476636,Math.toRadians(shoot_degree));



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
    private RedFarAutoIteration2.Paths paths; // Paths defined in the Paths class

    public enum States {
        Initial,
        Shoot,
        Pickup1,
        Backpick,
        Pickup2,
        Backpick2,
        Pickup3,
        Shoot2,
        Shoot3
    }
    public States current_state = States.Initial;

    public Pose startingPose = new Pose(88, 8, Math.toRadians(135));
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

        paths = new RedFarAutoIteration2.Paths(follower); // Build paths

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

    public static long fire_wait_time = 2500;
    public ElapsedTime shoot_timer = new ElapsedTime();
    public void prevent_false_shots() {
        if (current_state != States.Shoot && current_state != States.Shoot2 && current_state != States.Shoot3) {
            Launcher.shut_gates(robotComposition);
        }
    }

    public void move() {
        switch(current_state){
            case Initial:
                if(shoot_timer.milliseconds() >= 1000) {
                    intake.setPower(1);
                    follower.followPath(paths.Shoot);
                    current_state = States.Shoot;
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
                break;
            case Shoot:
                if (follower.atParametricEnd()) {
                    robotComposition.feeder.setPower(1);
                    if (shoot_timer.milliseconds() > fire_wait_time + 2000) {
                        follower.followPath(paths.Pickup1);
                        current_state = States.Pickup1;
                    } else {
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.90) {
                            Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.80) {
                            Launcher.launchLeft(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.70) {
                            Launcher.launchRight(true, robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                        if (shoot_timer.milliseconds() > fire_wait_time * 0.60) {
                            Launcher.spam(robotComposition, Launcher.LauncherDistance.FAR, 0);
                        }
                    }
                }
                break;
            case Backpick2:
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.Pickup3);
                    current_state = States.Pickup3;
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
                break;
            case Pickup3:
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.Shoot2);
                    current_state = States.Shoot2;
                }
            case Backpick:
                if(follower.atParametricEnd()){
                    follower.followPath(paths.Pickup2);
                    current_state = States.Pickup2;
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
                break;
            case Pickup1:
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.Backpick);
                    current_state = States.Backpick;
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
            case Pickup2:
                if (follower.atParametricEnd()) {
                    follower.followPath(paths.Backpick2);
                    current_state = States.Backpick2;
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
            case Shoot2:
                if(follower.atParametricEnd()){
                    robotComposition.feeder.setPower(1);
                    if(shoot_timer.milliseconds() > fire_wait_time) {
                        break;
                    } else{
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
        public PathChain Pickup1;
        public PathChain Backpick;
        public PathChain Backpick2;
        public PathChain Pickup3;
        public PathChain Pickup2;
        public PathChain Shoot2;

        public Paths(Follower follower) {
            Shoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 8.000),

                                    new Pose(83.393, 17.383)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(55))

                    .build();

            Pickup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.393, 17.383),

                                    new Pose(133.449, 9.720)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(0))

                    .build();

            Backpick = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.449, 9.720),

                                    new Pose(128.813, 10.187)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Backpick2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.449, 9.720),

                                    new Pose(128.813, 10.187)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Pickup2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.813, 10.187),

                                    new Pose(133.449, 9.720)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Pickup3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.813, 10.187),

                                    new Pose(133.449, 9.720)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.449, 9.720),

                                    new Pose(82.841, 17.421)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(55))

                    .build();
        }
    }

}
