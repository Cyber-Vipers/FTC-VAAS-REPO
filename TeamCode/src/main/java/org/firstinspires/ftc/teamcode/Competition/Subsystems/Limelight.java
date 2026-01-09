package org.firstinspires.ftc.teamcode.Competition.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Competition.Robot_Composition;
@Configurable
public class Limelight {
    public static void aim(Robot_Composition robot) {
        Limelight.look_up(robot);
        Limelight3A limelight = robot.limelight3A;
        LLResult result = limelight.getLatestResult();

        for (LLResultTypes.FiducialResult fid_res: result.getFiducialResults()) {
            if (fid_res.getFiducialId() == 20) {

            }
        }
    }
    public static void look_up(Robot_Composition robot) {
        robot.limelight_servo.setPosition(up_angle);
    }

    public static void look_down(Robot_Composition robot) {
        robot.limelight_servo.setPosition(down_angle);
    }

    public static void look_middle(Robot_Composition robot) {
        robot.limelight_servo.setPosition(middle_angle);
    }

    public static void read_april_tag(Robot_Composition robot) {
        robot.limelight_servo.setPosition(up_angle);
        Limelight3A limelight = robot.limelight3A;
        LLResult result = limelight.getLatestResult();
        if (limelight.pipelineSwitch(0)) {
            for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()) {
                if (fiducialResult.getFiducialId() == 20 || fiducialResult.getFiducialId() == 24) {
                    robot.follower.turnTo(Math.toRadians(fiducialResult.getTargetXDegrees()));
                }
            }
        }

    }


    public static void example(Robot_Composition robot) {
        robot.telemetry.addLine("Running Example");
        robot.limelight_servo.setPosition(0);
    }
    public static long horse_time = 0;
    public static long horse_wait = 1000;
    public static int horse_loop = 0;


    public static double up_angle = 0.2;
    public static double down_angle = 0;
    public static double middle_angle = 0.15;
    public static void all_knowing_horse(Robot_Composition robot) {
        if (horse_time == 0 && horse_loop == 0) {
            horse_time = System.currentTimeMillis();
            robot.limelight_servo.setPosition(down_angle);
            return;
        }
        if (horse_loop == 0 && horse_time +horse_wait <= System.currentTimeMillis()) {
            robot.limelight_servo.setPosition(middle_angle);
            horse_time = System.currentTimeMillis();
            horse_loop = 1;
        }
        if (horse_loop == 1 && horse_time +horse_wait <= System.currentTimeMillis()) {
            robot.limelight_servo.setPosition(up_angle);
            horse_time = System.currentTimeMillis();
            horse_loop = 2;
        }
        if (horse_loop == 2 && horse_time +horse_wait <= System.currentTimeMillis()) {
            robot.limelight_servo.setPosition(down_angle);
            horse_time = 0;
            horse_loop = 0;
        }
    }
}
