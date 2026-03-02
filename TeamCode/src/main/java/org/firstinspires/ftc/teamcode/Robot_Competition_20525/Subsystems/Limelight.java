package org.firstinspires.ftc.teamcode.Robot_Competition_20525.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Robot_Competition_20525.Util.Robot_Composition;
@Configurable
public class Limelight {

    public static double heading_offset = -30;

    public static double target_apriltag(Robot_Composition robot, int apriltag_id) {
        Limelight.look_up(robot);
        Limelight3A limelight = robot.limelight3A;
        LLResult result = limelight.getLatestResult();
        for (LLResultTypes.FiducialResult fid_res: result.getFiducialResults()) {
            if (fid_res.getFiducialId() == apriltag_id) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                double target_heading = Math.toDegrees(robot.follower.getHeading()-Math.toRadians(tx)) + heading_offset;
                return target_heading;
            }
        }
        return Math.toDegrees(robot.follower.getHeading());
    }

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
        look_up(robot);
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
