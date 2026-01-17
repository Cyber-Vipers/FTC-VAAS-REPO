package org.firstinspires.ftc.teamcode.Competition.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.Robot_Composition;
import org.firstinspires.ftc.teamcode.Competition.opmodes.Teleop.Competition_Teleop;

import static org.firstinspires.ftc.teamcode.Competition.Subsystems.Limelight.aim;

@Configurable
public class Launcher {
    public enum LauncherDistance {
        CLOSE,
        FAR;
    }
    public static double FEED_TIME_SECONDS = 1.0;
    public static double left_feeder_block = 0.25;
    public static double right_feeder_block = 0.5;
    public static double right_feeder_up = 0;
    public static double left_feeder_up = 0.5;
    public static final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    public static final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.
    public static final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    public static final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.

    public static double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    public static double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    public static ElapsedTime leftFeederTimer = new ElapsedTime();
    public static ElapsedTime rightFeederTimer = new ElapsedTime();
    public static LaunchState rightLaunchState = LaunchState.IDLE;
    public static LaunchState leftLaunchState = LaunchState.IDLE;
    //First is on the left, other two are on the right
    public static enum insides {
        PPG,
        PGP,
        GPP,
    }
    public static double first_wait = 0.55;
    public static double second_wait = 0.65;
    public static double third_wait = 0.80;
    public static boolean delagate_shots(long wait_time, long since_stop, Robot_Composition robot, insides inside,int obelisk,LauncherDistance launcherDistance) {
            switch (inside) {
                case PPG:
                    switch (obelisk) {
                        case 21:
                            //Impossible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0);
                            }
                            break;
                        case 22:
                        case 23:
                            //Impossible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0);
                            }
                            break;
                    }
                    break;
                case GPP:
                    switch (obelisk) {
                        case 21:
                            //Possible!
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0);
                            }
                            break;
                        case 22:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0);
                            }
                            break;
                        case 23:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchLeft(true,robot,launcherDistance,0);
                            }
                            break;
                    }
                    break;
                case PGP:
                    switch (obelisk) {
                        case 21:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0);
                            }
                            break;
                        case 22:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0);
                            }
                            break;
                        case 23:
                            //Impossible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,robot,launcherDistance,0);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,robot,launcherDistance,0);
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

    public static void spam(Robot_Composition robot, LauncherDistance launcherDistance,int modifier) {
        launchLeft(true,robot,launcherDistance,modifier);
        launchRight(true,robot,launcherDistance,modifier);
    }
    public static void launchRight(boolean shotRequested, Robot_Composition robot,LauncherDistance launcherDistance,float modifier) {

        if (launcherDistance.equals(LauncherDistance.FAR)) {
            launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY + modifier;
            launcherMin = LAUNCHER_FAR_MIN_VELOCITY + modifier;
        } else {
            launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY + modifier;
            launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY + modifier;
        }

        switch (rightLaunchState) {
            case IDLE:
                if (shotRequested) {
                    rightLaunchState = LaunchState.SPIN_UP;
                    robot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.intake.setPower(1);
                }
                break;
            case SPIN_UP:
                robot.leftLauncher.setVelocity(launcherTarget);
                robot.rightLauncher.setVelocity(launcherTarget);
                if (robot.leftLauncher.getVelocity() > launcherMin) {
                    rightLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.rightFeeder.setPosition(right_feeder_up);
                robot.feeder.setPower(1);
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
    public static void launchLeft(boolean shotRequested, Robot_Composition robot,LauncherDistance launcherDistance, float modifier) {

        if (launcherDistance.equals(LauncherDistance.FAR)) {
            launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
            launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
        } else {
            launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
            launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
        }

        switch (leftLaunchState) {
            case IDLE:
                if (shotRequested) {
                    leftLaunchState = LaunchState.SPIN_UP;
                    robot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.intake.setPower(1);
                }
                break;
            case SPIN_UP:
                robot.leftLauncher.setVelocity(launcherTarget);
                robot.rightLauncher.setVelocity(launcherTarget);
                if (robot.leftLauncher.getVelocity() > launcherMin) {
                    leftLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.leftFeeder.setPosition(left_feeder_up);
                robot.feeder.setPower(1);
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

    public static double get_launcher_reg(Robot_Composition composition){
        Follower follower = composition.follower;
        Limelight3A limelight3A = composition.limelight3A;
        LLResult result = limelight3A.getLatestResult();
        Pose3D current_position = result.getBotpose();
        double distance = Math.sqrt((current_position.getPosition().x - 13) + (current_position.getPosition().y - 134));
        return (Math.pow(-0.168269*distance,2)-(0.432692*distance)+1200) + (Math.pow(0.00862664*distance,2)-(5.6332*distance)+1200);

    }
}
