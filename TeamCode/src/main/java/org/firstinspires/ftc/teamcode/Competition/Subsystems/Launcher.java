package org.firstinspires.ftc.teamcode.Competition.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.opmodes.Teleop.Competition_Teleop;

@Configurable
public class Launcher {
    public enum LauncherDistance {
        CLOSE,
        FAR;
    }
    public static final double FEED_TIME_SECONDS = 1;
    public static double left_feeder_block = 0.25;
    public static double right_feeder_block = 0.25;
    public static double right_feeder_up = 0;
    public static double left_feeder_up = 1;
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
    public static boolean delagate_shots(long wait_time, long since_stop, insides inside, DcMotorEx leftLauncher, DcMotorEx rightLauncher, DcMotor feeder, Servo rightFeeder, Servo leftFeeder, int obelisk,LauncherDistance launcherDistance) {
            switch (inside) {
                case PPG:
                    switch (obelisk) {
                        case 21:
                            //Impossible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            break;
                        case 22:
                        case 23:
                            //Impossible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            break;
                    }
                    break;
                case GPP:
                    switch (obelisk) {
                        case 21:
                            //Possible!
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            break;
                        case 22:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            break;
                        case 23:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            break;
                    }
                    break;
                case PGP:
                    switch (obelisk) {
                        case 21:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            break;
                        case 22:
                            //possible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            break;
                        case 23:
                            //Impossible for perfect motif
                            if (since_stop + (wait_time * first_wait)  <= System.currentTimeMillis()) {
                                launchLeft(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * second_wait) <= System.currentTimeMillis()) {
                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            if (since_stop + (wait_time * third_wait) <= System.currentTimeMillis()) {

                                launchRight(true,leftLauncher,rightLauncher,feeder,rightFeeder,leftFeeder,launcherDistance);
                            }
                            break;
                    }
                    break;

            }
        return since_stop + wait_time <= System.currentTimeMillis();

    }

    public static void launchRight(boolean shotRequested, DcMotorEx leftLauncher, DcMotorEx rightLauncher, DcMotor feeder, Servo rightFeeder, Servo leftFeeder,LauncherDistance launcherDistance) {
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
                rightFeeder.setPosition(right_feeder_up);
                feeder.setPower(1);
                rightFeederTimer.reset();
                rightLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    feeder.setPower(0);
                    rightLaunchState = LaunchState.IDLE;
                    rightFeeder.setPosition(right_feeder_block);
                }
                break;
        }
    }
    public static void launchLeft(boolean shotRequested, DcMotorEx leftLauncher, DcMotorEx rightLauncher, DcMotor feeder, Servo rightFeeder, Servo leftFeeder, LauncherDistance launcherDistance) {
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
                leftFeeder.setPosition(left_feeder_up);
                feeder.setPower(1);
                leftFeederTimer.reset();
                leftLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    feeder.setPower(0);
                    leftLaunchState = LaunchState.IDLE;
                    leftFeeder.setPosition(left_feeder_block);

                }
                break;
        }
    }
}
