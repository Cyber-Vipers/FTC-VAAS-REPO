package org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Util.Robot_Composition;

@Configurable
public class Launcher {
    public enum LauncherDistance {
        CLOSE,
        FAR
    }
    
    public static double last_right_speed = 0;
    public static double last_left_speed = 0;
    public static double FEED_TIME_SECONDS = 1.5;
    public static double left_feeder_block = 0.3;
    public static double right_feeder_block = 0.65;
    //lets try and not interupt him too badly, as he is lecturing
    //we arent that luod
    public static double right_feeder_up = 0;
    public static double left_feeder_up = 1;
    public static double LAUNCHER_FAR_MAX_VELOCITY = 1375;
    public static final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    public static final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    public static double LAUNCHER_CLOSE_MAX_VELOCITY = 1140;
    public static double LAUNCHER_CLOSE_TARGET_VELOCITY = 1000; //in ticks/second for the close goal.
    public static double LAUNCHER_CLOSE_MIN_VELOCITY = 1000 ; //minimum required to start a shot for close goal.

    public static double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    public static double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
    public static double launcherMax = LAUNCHER_CLOSE_MAX_VELOCITY;
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    public static ElapsedTime leftFeederTimer = new ElapsedTime();
    public static ElapsedTime rightFeederTimer = new ElapsedTime();
    static LaunchState rightLaunchState = LaunchState.IDLE;
    static LaunchState leftLaunchState = LaunchState.IDLE;
    //First is on the left, other two are on the right
    public enum insides {
        PPG,
        PGP,
        GPP,
    }
    public static double first_wait = 0.55;
    public static double second_wait = 0.65;
    public static double third_wait = 0.80;

    public static boolean noshoot = false;
    public static boolean auto_active = false;

    public static boolean delegate_shots(long wait_time, long since_stop, Robot_Composition robot, insides inside, int obelisk, LauncherDistance launcherDistance) {
            switch (inside) {
                case PPG:
                    switch (obelisk) {
                        case 21:
                            //Impossible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0,false, 0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            break;
                        case 22:
                        case 23:
                            //Impossible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0,false, 0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            break;
                    }
                    break;
                case GPP:
                    switch (obelisk) {
                        case 21:
                            //Possible!
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0,false, 0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            break;
                        case 22:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0,false, 0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            break;
                        case 23:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchLeft(true,robot,launcherDistance,0,false, 0);
                            }
                            break;
                    }
                    break;
                case PGP:
                    switch (obelisk) {
                        case 21:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0,false, 0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            break;
                        case 22:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0,false, 0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            break;
                        case 23:
                            //Impossible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0,false, 0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0, false, 0);
                            }
                            break;
                    }
                    break;

            }
        return since_stop + wait_time <= System.currentTimeMillis();

    }



    public static void shut_gates(Robot_Composition robot) {
        robot.leftFeeder.setPosition(left_feeder_block);
        robot.rightFeeder.setPosition(right_feeder_block);
    }
    public static void open_gates(Robot_Composition robot) {
        robot.leftFeeder.setPosition(left_feeder_up);
        robot.rightFeeder.setPosition(right_feeder_up);
    }
    
    public static double feeder_speed = 0.55;

    public static void spinup(Robot_Composition robot, LauncherDistance launcherDistance, int modifier) {
        if (launcherDistance.equals(LauncherDistance.FAR)) {
            launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY + modifier;
            launcherMin = LAUNCHER_FAR_MIN_VELOCITY + modifier;
        } else {
            launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY + modifier;
            launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY + modifier;
        }
        robot.leftLauncher.setVelocity(launcherTarget);
        robot.rightLauncher.setVelocity(launcherTarget);
    }

    // public static ElapsedTime spam_timer = new ElapsedTime();
    // public static long wait_time = 250;
    public static float spam_modifier = 200;
    public static double distance_to_goal(Pose pose,Pose goalPosition) {
        //sqrt((x2-x1)^2+(y2-y1)^2)
        return Math.sqrt(Math.pow(goalPosition.getX()-pose.getX(),2)+Math.pow(goalPosition.getY()-pose.getY(),2));
    }


    //GET THE MAGIC NUMBERS FROM DESMOS ! ! !
    public static double linearFactor1 = 3.82104;
    public static double linearFactor2 = 967.9013;
    // public static double quadraticFactor3 = 1237.14761;
    public static double linearRegression(Pose pose, Pose goalPosition) {
        double dist = distance_to_goal(pose,goalPosition);
        return (linearFactor1 *dist + linearFactor2);
    }
    public static double quarticFactor1 = 0.000270501;
    public static double quarticFactor2 = 0.0920206;
    public static double quarticFactor3 = 11.41175;
    public static double quarticFactor4 = 607.57568;
    public static double quarticFactor5 = 12902.1395;


    public double quartic_regression(Pose pose,Pose goalPosition) {
        double dist = distance_to_goal(pose,goalPosition);
        return Math.pow(quarticFactor1*dist,4) - Math.pow(quarticFactor2*dist,3) + Math.pow(quarticFactor3*dist,2) + quarticFactor4*dist + quarticFactor5;
    }
    public static void spam(Robot_Composition robot, LauncherDistance launcherDistance,int modifier) {
        launchLeft(true,robot,launcherDistance,modifier);
        launchRight(true,robot,launcherDistance,modifier);
        launchRight(true,robot,launcherDistance,modifier);
    }
    public static void launchRight(boolean shotRequested, Robot_Composition robot,LauncherDistance launcherDistance,float modifier) {

        last_right_speed = robot.rightLauncher.getVelocity();
        if (launcherDistance.equals(LauncherDistance.FAR)) {
            launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY + modifier;
            launcherMax = LAUNCHER_FAR_MAX_VELOCITY + modifier;
            launcherMin = LAUNCHER_FAR_MIN_VELOCITY + modifier;

        } else {
            launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY + modifier;
            launcherMax = LAUNCHER_CLOSE_MAX_VELOCITY + modifier;
            launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY + modifier;
        }
        if (!noshoot && auto_active) {
            switch (rightLaunchState) {
                case IDLE:
                    if (shotRequested) {
                        robot.leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE   );
                        robot.rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
                        rightLaunchState = LaunchState.SPIN_UP;
                        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.intake.setPower(1);
                    }
                    break;
                case SPIN_UP:
                    robot.leftLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setPower(1);
                    
                    if (robot.rightLauncher.getVelocity() > launcherMin && robot.rightLauncher.getVelocity() < launcherMax) {
                        rightLaunchState = LaunchState.LAUNCH;
                    }
                    
                    break;
                case LAUNCH:
                    robot.rightFeeder.setPosition(right_feeder_up);
                    robot.feeder.setPower(feeder_speed);

                    rightFeederTimer.reset();
                    rightLaunchState = LaunchState.LAUNCHING;
                    break;
                case LAUNCHING:
                    if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                        robot.feeder.setPower(0);
                        rightLaunchState = LaunchState.IDLE;
                        robot.rightFeeder.setPosition(right_feeder_block);
                    }
                    robot.right_inside = null;
                    break;
            }
        } else if (!auto_active){
            switch (rightLaunchState) {
                case IDLE:
                    if (shotRequested) {
                        robot.leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE   );
                        robot.rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
                        rightLaunchState = LaunchState.SPIN_UP;
                        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.intake.setPower(1);
                    }
                    break;
                case SPIN_UP:
                    robot.leftLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setPower(1);
                    
                    if (robot.rightLauncher.getVelocity() > launcherMin && robot.rightLauncher.getVelocity() < launcherMax) {
                        rightLaunchState = LaunchState.LAUNCH;
                    }
                    break;
                case LAUNCH:
                    robot.rightFeeder.setPosition(right_feeder_up);
                    robot.feeder.setPower(feeder_speed);

                    rightFeederTimer.reset();
                    rightLaunchState = LaunchState.LAUNCHING;
                    break;
                case LAUNCHING:
                    if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                        robot.feeder.setPower(0);
                        rightLaunchState = LaunchState.IDLE;
                        robot.rightFeeder.setPosition(right_feeder_block);
                    }
                    robot.right_inside = null;
                    break;
            }
        }

    }

    public static void launchRight(boolean shotRequested, Robot_Composition robot,LauncherDistance launcherDistance,float modifier,boolean overide, double override_value) {
        last_right_speed = robot.rightLauncher.getVelocity();
        if (launcherDistance.equals(LauncherDistance.FAR)) {
            launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY + modifier;
            launcherMax = LAUNCHER_FAR_MAX_VELOCITY + modifier;
            launcherMin = LAUNCHER_FAR_MIN_VELOCITY + modifier;

        } else {
            launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY + modifier;
            launcherMax = LAUNCHER_CLOSE_MAX_VELOCITY + modifier;
            launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY + modifier;
        }
        if (overide) {
            launcherTarget = override_value;
            launcherMin = override_value - 50;
            launcherMax = override_value + 50;
        }

        if (!noshoot && auto_active) {
        switch (rightLaunchState) {
            case IDLE:
                if (shotRequested) {
                    robot.leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE   );
                    robot.rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
                    rightLaunchState = LaunchState.SPIN_UP;
                    robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.intake.setPower(1);
                }
                break;
            case SPIN_UP:
                robot.leftLauncher.setVelocity(launcherTarget);
                robot.rightLauncher.setVelocity(launcherTarget);
                robot.rightLauncher.setPower(1);
                
                if (robot.rightLauncher.getVelocity() > launcherMin  + 25 && robot.rightLauncher.getVelocity() < launcherMax) {
                    rightLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.feeder.setPower(feeder_speed);
                robot.rightFeeder.setPosition(right_feeder_up);

                rightFeederTimer.reset();
                rightLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    robot.feeder.setPower(0);
                    rightLaunchState = LaunchState.IDLE;
                    robot.rightFeeder.setPosition(right_feeder_block);
                }
                robot.right_inside = null;
                break;
        }} else if (!auto_active) {
            switch (rightLaunchState) {
                case IDLE:
                    if (shotRequested) {
                        robot.leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE   );
                        robot.rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
                        rightLaunchState = LaunchState.SPIN_UP;
                        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.intake.setPower(1);
                    }
                    break;
                case SPIN_UP:
                    robot.leftLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setPower(1);
                    
                    if (robot.rightLauncher.getVelocity() > launcherMin  + 25 && robot.rightLauncher.getVelocity() < launcherMax) {
                        rightLaunchState = LaunchState.LAUNCH;
                    }
                    break;
                case LAUNCH:
                    robot.rightFeeder.setPosition(right_feeder_up);
                    robot.feeder.setPower(feeder_speed);


                    rightFeederTimer.reset();
                    rightLaunchState = LaunchState.LAUNCHING;
                    break;
                case LAUNCHING:
                    if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                        robot.feeder.setPower(0);
                        rightLaunchState = LaunchState.IDLE;
                        robot.rightFeeder.setPosition(right_feeder_block);
                    }
                    robot.right_inside = null;
                    break;
        }
    }
    }
    public static void launchLeft(boolean shotRequested, Robot_Composition robot,LauncherDistance launcherDistance, float modifier, boolean overide, double override_value) {
        last_left_speed = robot.rightLauncher.getVelocity();
        if (launcherDistance.equals(LauncherDistance.FAR)) {
            launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY + modifier;
            launcherMax = LAUNCHER_FAR_MAX_VELOCITY + modifier;
            launcherMin = LAUNCHER_FAR_MIN_VELOCITY + modifier;
        } else {
            launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY + modifier;
            launcherMax = LAUNCHER_CLOSE_MAX_VELOCITY + modifier;
            launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY + modifier;
        }
        if (overide) {
            launcherTarget = override_value;
            launcherMin = override_value - 25;
            launcherMax = override_value + 25;
        }
        if (!noshoot && auto_active) {
        switch (leftLaunchState) {
            case IDLE:
                if (shotRequested) {
                    robot.leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE   );
                    robot.rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
                    leftLaunchState = LaunchState.SPIN_UP;
                    robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.intake.setPower(1);
                }
                break;
            case SPIN_UP:
                robot.leftLauncher.setVelocity(launcherTarget);
                robot.rightLauncher.setVelocity(launcherTarget);
                robot.rightLauncher.setPower(1);
                
                if (robot.rightLauncher.getVelocity() > launcherMin && robot.rightLauncher.getVelocity() < launcherMax) {
                    leftLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.leftFeeder.setPosition(left_feeder_up);
                robot.feeder.setPower(feeder_speed);

                leftFeederTimer.reset();
                leftLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    robot.feeder.setPower(0);
                    leftLaunchState = LaunchState.IDLE;
                    robot.leftFeeder.setPosition(left_feeder_block);

                }
                robot.left_inside = null;
                break;
        }
        } else if (!auto_active) {
            switch (leftLaunchState) {
                case IDLE:
                    if (shotRequested) {
                        robot.leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE   );
                        robot.rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
                        leftLaunchState = LaunchState.SPIN_UP;
                        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.intake.setPower(1);
                    }
                    break;
                case SPIN_UP:
                    robot.leftLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setPower(1);
                    
                    if (robot.rightLauncher.getVelocity() > launcherMin && robot.rightLauncher.getVelocity() < launcherMax) {
                        leftLaunchState = LaunchState.LAUNCH;
                    }
                    break;
                case LAUNCH:
                    robot.leftFeeder.setPosition(left_feeder_up);
                    robot.feeder.setPower(feeder_speed);

                    leftFeederTimer.reset();
                    leftLaunchState = LaunchState.LAUNCHING;
                    break;
                case LAUNCHING:
                    if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                        robot.feeder.setPower(0);
                        leftLaunchState = LaunchState.IDLE;
                        robot.leftFeeder.setPosition(left_feeder_block);

                    }
                    robot.left_inside = null;
                    break;
            }
        }
    }




    public static void launchLeft(boolean shotRequested, Robot_Composition robot,LauncherDistance launcherDistance, float modifier) {
        last_left_speed = robot.rightLauncher.getVelocity();
        if (launcherDistance.equals(LauncherDistance.FAR)) {
            launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY + modifier;
            launcherMax = LAUNCHER_FAR_MAX_VELOCITY + modifier;
            launcherMin = LAUNCHER_FAR_MIN_VELOCITY + modifier;
        } else {
            launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY + modifier;
            launcherMax = LAUNCHER_CLOSE_MAX_VELOCITY + modifier;
            launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY + modifier;
        }
        if (!noshoot && auto_active) {
            switch (leftLaunchState) {
                case IDLE:
                    if (shotRequested) {
                        robot.leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE   );
                        robot.rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
                        leftLaunchState = LaunchState.SPIN_UP;
                        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.intake.setPower(1);
                    }
                    break;
                case SPIN_UP:
                    robot.leftLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setPower(1);
                    
                    if (robot.rightLauncher.getVelocity() > launcherMin && robot.rightLauncher.getVelocity() < launcherMax) {
                        leftLaunchState = LaunchState.LAUNCH;
                    }
                    break;
                case LAUNCH:
                    robot.feeder.setPower(feeder_speed);
                    robot.leftFeeder.setPosition(left_feeder_up);
                    leftFeederTimer.reset();
                    leftLaunchState = LaunchState.LAUNCHING;
                    break;
                case LAUNCHING:
                    if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                        robot.feeder.setPower(0);
                        leftLaunchState = LaunchState.IDLE;
                        robot.leftFeeder.setPosition(left_feeder_block);

                    }
                    robot.left_inside = null;
                    break;
            }
        } else if (!auto_active) {
            switch (leftLaunchState) {
                case IDLE:
                    if (shotRequested) {
                        robot.leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE   );
                        robot.rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
                        leftLaunchState = LaunchState.SPIN_UP;
                        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.intake.setPower(1);
                    }
                    break;
                case SPIN_UP:
                    robot.leftLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setVelocity(launcherTarget);
                    robot.rightLauncher.setPower(1);
                    
                    if (robot.rightLauncher.getVelocity() > launcherMin && robot.rightLauncher.getVelocity() < launcherMax) {
                        leftLaunchState = LaunchState.LAUNCH;
                    }
                    break;
                case LAUNCH:
                    robot.leftFeeder.setPosition(left_feeder_up);
                    robot.feeder.setPower(feeder_speed);

                    leftFeederTimer.reset();
                    leftLaunchState = LaunchState.LAUNCHING;
                    break;
                case LAUNCHING:
                    if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                        robot.feeder.setPower(0);
                        leftLaunchState = LaunchState.IDLE;
                        robot.leftFeeder.setPosition(left_feeder_block);

                    }
                    robot.left_inside = null;
                    break;
            }
        }
    }


    public static double distance_to_goal(Robot_Composition robot,boolean is_blue) {
        Pose pose = robot.follower.getPose();
        //Goal Position 10,139
        if (is_blue) {
            return new Pose(10,139).distanceFrom(robot.follower.getPose());
        } else {
            return new Pose(134,139).distanceFrom(robot.follower.getPose());
        }
    }

//    public static double get_launcher_reg(Robot_Composition composition){
//        Follower follower = composition.follower;
//        Limelight3A limelight3A = composition.limelight3A;
//        LLResult result = limelight3A.getLatestResult();
//        Pose3D current_position = result.getBotpose();
//        double distance = Math.sqrt((current_position.getPosition().x - 13) + (current_position.getPosition().y - 134));
//        return (Math.pow(-0.168269*distance,2)-(0.432692*distance)+1200) + (Math.pow(0.00862664*distance,2)-(5.6332*distance)+1200);
//
//    }
}
