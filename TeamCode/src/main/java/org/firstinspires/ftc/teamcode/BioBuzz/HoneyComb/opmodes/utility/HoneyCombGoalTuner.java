package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.FieldConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.GoalAim;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.LiftShooter;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.MathFunc;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Finds the goal's field position by "sighting" it from a few spots instead of measuring to its
 * center. Each sighting is a line from the robot along the shooter's direction; the goal is where
 * those lines cross (least squares, so 3+ sightings average out aiming error).
 *
 *   1. Put the robot on the reset spot and press Back to set odometry there.
 *   2. Drive somewhere, then turn (LB for slow) until the shooter points straight at the goal
 *      center. Press A to record the sighting.
 *   3. Repeat from 2-4 more spots around the goal, 30+ deg apart for a good fix.
 *   4. Copy the printed line into FieldConstants.
 *   5. Optional check: B uses the estimate for this run; then hold RB anywhere and the robot
 *      should turn to face the goal.
 *
 * gamepad1: sticks drive, LB slow, RB hold = auto-aim at the current goal, A record,
 * Y delete last, B use the estimate, Back reset odometry. During Init: X = blue, B = red.
 */
@TeleOp(name = "HoneyComb Goal Tuner", group = "Honeycomb Tuning")
public class HoneyCombGoalTuner extends LinearOpMode {
    /** Warn when the sightings span less than this; nearly parallel lines give a poor fix. */
    public static double MIN_SPREAD_DEG = 20;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1);
        NeuronLocalizer localizer = LiftShooter.createLocalizer(hardwareMap, telemetry);
        localizer.calibrate(false);
        GoalAim aim = new GoalAim(drive, localizer);
        FieldConstants.Alliance alliance = FieldConstants.Alliance.RED;

        List<Pose2D> sightings = new ArrayList<>();
        Pose2D estimate = null;
        String status = "";

        while (opModeInInit()) {
            if (gamepad1.xWasPressed()) alliance = FieldConstants.Alliance.BLUE;
            if (gamepad1.bWasPressed()) alliance = FieldConstants.Alliance.RED;
            telemetry.addLine("X = blue, B = red");
            telemetry.addData("alliance", alliance);
            telemetry.update();
        }

        while (opModeIsActive()) {
            Pose2D pose = localizer.getPose(); // the one Pinpoint read this loop

            if (gamepad1.backWasPressed()) {
                localizer.setPosition(FieldConstants.resetPose(alliance));
                pose = FieldConstants.resetPose(alliance);
                status = sightings.isEmpty() ? "odometry reset" : "odometry reset -- old sightings cleared";
                sightings.clear();
                estimate = null;
            }

            if (gamepad1.right_bumper) {
                aim.update(pose, FieldConstants.headingToGoal(pose, alliance));
            } else {
                aim.stop();
            }
            drive.update();

            if (gamepad1.aWasPressed()) {
                sightings.add(new Pose2D(pose.x, pose.y, pose.h));
                estimate = intersect(sightings);
                status = "recorded sighting " + sightings.size();
            }
            if (gamepad1.yWasPressed() && !sightings.isEmpty()) {
                sightings.remove(sightings.size() - 1);
                estimate = intersect(sightings);
                status = "deleted last sighting";
            }
            if (gamepad1.bWasPressed()) {
                if (estimate == null) {
                    status = "no estimate yet -- need 2+ sightings from different directions";
                } else {
                    FieldConstants.setGoal(alliance, estimate);
                    status = "estimate in use for this run -- hold RB to check the aim";
                }
            }

            Pose2D goal = FieldConstants.goal(pose, alliance);
            double aimErrorDeg = Math.toDegrees(MathFunc.angleWrap(FieldConstants.headingToGoal(pose, alliance) - pose.h));
            telemetry.addData("alliance", alliance);
            telemetry.addData("status", status);
            telemetry.addData("pose", "x %.1f  y %.1f  h %.1f°", pose.x, pose.y, Math.toDegrees(pose.h));
            telemetry.addData("current goal", "x %.1f  y %.1f  (%s CELL)", goal.x, goal.y,
                    FieldConstants.targetCell(pose, alliance));
            telemetry.addData("to current goal", "%.1f in, turn %.1f°",
                    FieldConstants.distanceToGoal(pose, alliance), aimErrorDeg);
            telemetry.addLine();
            telemetry.addData("sightings", sightings.size());
            for (int i = 0; i < sightings.size(); i++) {
                Pose2D s = sightings.get(i);
                telemetry.addLine(String.format(Locale.US, "  %d: x %.1f  y %.1f  aim %.1f°",
                        i + 1, s.x, s.y, Math.toDegrees(aimDirection(s))));
            }
            if (sightings.size() >= 2 && spreadDeg(sightings) < MIN_SPREAD_DEG) {
                telemetry.addLine(String.format(Locale.US,
                        "  !! sightings only %.0f° apart -- add one from a more different angle", spreadDeg(sightings)));
            }
            if (estimate != null) {
                telemetry.addData("estimate", "x %.1f  y %.1f", estimate.x, estimate.y);
                if (sightings.size() >= 3) {
                    telemetry.addData("miss (RMS)", "%.1f in -- lines' average distance from the estimate", estimate.h);
                }
                telemetry.addLine(String.format(Locale.US,
                        "copy into FieldConstants:  HIVE_SIDE_IN = %.1f;  CELL_OFFSET_IN = %.1f;  (%s CELL was up)",
                        Math.abs(estimate.y), Math.abs(estimate.x), estimate.x >= 0 ? "SCORING" : "AUDIENCE"));
            } else if (sightings.size() >= 2) {
                telemetry.addLine("estimate: none -- sightings are (nearly) parallel");
            }
            telemetry.addLine();
            telemetry.addLine("LB slow | RB aim | A record | Y undo | B use estimate | Back reset");
            telemetry.update();
        }
    }

    /** Field direction the shooter fires along for a sighting taken at this pose. */
    private static double aimDirection(Pose2D sighting) {
        return sighting.h + FieldConstants.SHOOTER_HEADING_OFFSET_RAD;
    }

    /**
     * Least-squares intersection of the sighting lines: the point with the smallest sum of
     * squared perpendicular distances to every line. Null with fewer than 2 sightings or when the
     * lines are (nearly) parallel. The returned pose's h holds the RMS distance (in) of the lines.
     */
    private static Pose2D intersect(List<Pose2D> sightings) {
        if (sightings.size() < 2) return null;
        double a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
        for (Pose2D s : sightings) {
            double dx = Math.cos(aimDirection(s)), dy = Math.sin(aimDirection(s));
            double m11 = 1 - dx * dx, m12 = -dx * dy, m22 = 1 - dy * dy;
            a11 += m11; a12 += m12; a22 += m22;
            b1 += m11 * s.x + m12 * s.y;
            b2 += m12 * s.x + m22 * s.y;
        }
        double det = a11 * a22 - a12 * a12;
        if (Math.abs(det) < 1e-3) return null;
        double x = (a22 * b1 - a12 * b2) / det;
        double y = (a11 * b2 - a12 * b1) / det;

        double sumSq = 0;
        for (Pose2D s : sightings) {
            double dx = Math.cos(aimDirection(s)), dy = Math.sin(aimDirection(s));
            double perp = (x - s.x) * dy - (y - s.y) * dx;
            sumSq += perp * perp;
        }
        return new Pose2D(x, y, Math.sqrt(sumSq / sightings.size()));
    }

    /** Largest angle (degrees, 0-90) between any two sighting lines. */
    private static double spreadDeg(List<Pose2D> sightings) {
        double best = 0;
        for (int i = 0; i < sightings.size(); i++) {
            for (int j = i + 1; j < sightings.size(); j++) {
                double diff = Math.abs(Math.toDegrees(MathFunc.angleWrap(
                        aimDirection(sightings.get(i)) - aimDirection(sightings.get(j)))));
                best = Math.max(best, Math.min(diff, 180 - diff));
            }
        }
        return best;
    }
}
