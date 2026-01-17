package org.firstinspires.ftc.teamcode.Competition.opmodes.auto;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Competition.Robot_Composition;
import org.firstinspires.ftc.teamcode.Competition.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

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

    public Pose startingPose = new Pose(21.981, 124.935, Math.toRadians(135));
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
        robotComposition = new Robot_Composition(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, leftLauncher, rightLauncher, intake, feeder, leftFeeder, rightFeeder,color1,color2,follower,telemetry,diverter,limelight_servo,limelight);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        panelsField.update();
        move();
        prevent_false_shots();
        draw();
        // Log values to Panels and Driver Station
        telemetry.addData("Current Path", current_state.name());
        panelsTelemetry.debug("Current Path", current_state.name());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        telemetry.update();
        panelsTelemetry.update(telemetry);

    }
    public void draw() {
        Drawing.drawDebug(follower);
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
                    if (shoot_timer.milliseconds() > fire_wait_time) {
                        follower.followPath(paths.Intake1);
                        current_state = States.Intake1;
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

            case Intake1:
                if(follower.atParametricEnd()){
                    follower.followPath(paths.Shoot2);
                    current_state = States.Shoot2;
                    shoot_timer.reset();
                    shoot_timer.startTime();
                }
                break;
            case Shoot2:
                if(follower.atParametricEnd()){
                    if(shoot_timer.milliseconds() > fire_wait_time) {
                        follower.followPath(paths.Intake2);
                        current_state = States.Intake2;
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
            case Intake2:
                if(follower.atParametricEnd()){
                    follower.followPath(paths.Shoot3);
                    current_state = States.Shoot3;
                    break;
                }
                break;
        }
    }


    public static class Paths {
        public PathChain Shoot;
        public PathChain prepareIntake1;
        public PathChain Intake1;
        public PathChain Shoot2;
        public PathChain Intake2;
        public PathChain Shoot3;
        public PathChain Path6;
        public Paths(Follower follower) {
            Shoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80.500, 8.000),

                                    new Pose(82.393, 17.383)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                    .build();

            prepareIntake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.393, 17.383),

                                    new Pose(102.804, 36.047)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.804, 36.047),

                                    new Pose(136.495, 36.047)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.495, 36.047),

                                    new Pose(82.393, 17.383)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                    .build();

            Intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.393, 17.383),

                                    new Pose(129.916, 9.299)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.916, 9.299),

                                    new Pose(82.393, 17.383)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                    .build();
        }
    }

}
