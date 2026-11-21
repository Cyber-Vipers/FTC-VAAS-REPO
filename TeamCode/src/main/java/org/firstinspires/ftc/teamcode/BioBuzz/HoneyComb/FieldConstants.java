package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.neuronpathing.MathFunc;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Field positions, in the same coordinates NeuronLocalizer reports (inches, radians).
 *
 * Field frame: the Neuron Path Planner's, so poses drawn there can go straight into odometry.
 * Origin at the field center:
 *   +x  away from the audience (toward the "scoring" side of the HIVE)
 *   +y  toward the RED alliance wall; the blue wall is at y = -HALF_FIELD_IN
 *   heading 0 = facing +x, counter-clockwise positive
 * Red and blue mirror across the x axis (y -> -y, heading -> -heading).
 *
 * Goals: the HIVE Structure sits in the middle of the field (Competition Manual 9.6). It holds a
 * red HIVE on the red side and a blue HIVE on the blue side. Each HIVE has two CELLS about
 * 18.8 in apart, one toward the audience and one toward the scoring side, and tips so one CELL
 * faces up. With AUTO_CELL (the default) the robot aims at the CELL on its own half of the field.
 * HIVE_SIDE_IN is an estimate: measure the real position with the HoneyComb Goal Tuner.
 */
@Configurable
public class FieldConstants {
    public enum Alliance { RED, BLUE }
    public enum Cell { SCORING, AUDIENCE }

    /** Field center to the inside of a perimeter wall. */
    public static double HALF_FIELD_IN = 70.5;

    /** Each HIVE's pivot: red at y = +HIVE_SIDE_IN, blue at y = -HIVE_SIDE_IN, both on x = 0. */
    public static double HIVE_SIDE_IN = 13.5;
    /** Each CELL's distance from its HIVE's pivot, toward the scoring side (+x) or audience side (-x). */
    public static double CELL_OFFSET_IN = 18.8 / 2;

    /**
     * true: aim at the CELL on the robot's half of the field (scoring CELL when x > 0, audience
     * CELL when x < 0), so nobody has to say which is up. false: aim at redUpCell / blueUpCell.
     */
    public static boolean AUTO_CELL = true;
    /** With AUTO_CELL, the robot must be this far past x = 0 before the target switches CELLs. */
    public static double CELL_SWITCH_BAND_IN = 3;
    public static Cell redUpCell = Cell.SCORING;
    public static Cell blueUpCell = Cell.SCORING;
    private static Cell redAutoCell = Cell.SCORING, blueAutoCell = Cell.SCORING;

    /**
     * Reset spot (Back button): the alliance's loading-zone corner, back against its own alliance
     * wall, side against the end wall, facing the other alliance's wall. Measure your robot:
     * half its length (front to back) and half its width put its center that far off the walls.
     */
    public static double ROBOT_HALF_LENGTH_IN = 9, ROBOT_HALF_WIDTH_IN = 9;

    /** Direction the shooter fires relative to the robot's front, in radians (Math.PI = out the back). */
    public static double SHOOTER_HEADING_OFFSET_RAD = 0;

    public static Pose2D resetPose(Alliance alliance) {
        double x = HALF_FIELD_IN - ROBOT_HALF_WIDTH_IN;
        double y = HALF_FIELD_IN - ROBOT_HALF_LENGTH_IN;
        // RED: far end of the red wall, facing the blue wall. BLUE: audience end of the blue wall, facing
        // red. The two loading-zone corners are diagonal from each other, so these are NOT mirror images
        // across the x axis. If your reset spot is somewhere else, change it here.
        return alliance == Alliance.RED ? new Pose2D(x, y, -Math.PI / 2) : new Pose2D(-x, -y, Math.PI / 2);
    }

    /** The CELL to shoot at from here (see AUTO_CELL). */
    public static Cell targetCell(Pose2D robot, Alliance alliance) {
        if (!AUTO_CELL) return alliance == Alliance.RED ? redUpCell : blueUpCell;
        Cell cell = alliance == Alliance.RED ? redAutoCell : blueAutoCell;
        if (robot.x > CELL_SWITCH_BAND_IN) cell = Cell.SCORING;
        else if (robot.x < -CELL_SWITCH_BAND_IN) cell = Cell.AUDIENCE;
        if (alliance == Alliance.RED) redAutoCell = cell; else blueAutoCell = cell;
        return cell;
    }

    public static Pose2D cellPosition(Cell cell, Alliance alliance) {
        double x = cell == Cell.SCORING ? CELL_OFFSET_IN : -CELL_OFFSET_IN;
        double y = alliance == Alliance.RED ? HIVE_SIDE_IN : -HIVE_SIDE_IN;
        return new Pose2D(x, y, 0);
    }

    /** The goal to shoot at from here, field frame. */
    public static Pose2D goal(Pose2D robot, Alliance alliance) {
        return cellPosition(targetCell(robot, alliance), alliance);
    }

    /**
     * Use a goal position measured by the Goal Tuner until the app restarts. The field is
     * symmetric, so this sets HIVE_SIDE_IN / CELL_OFFSET_IN for both alliances.
     */
    public static void setGoal(Alliance alliance, Pose2D measured) {
        HIVE_SIDE_IN = Math.abs(measured.y);
        CELL_OFFSET_IN = Math.abs(measured.x);
        Cell up = measured.x >= 0 ? Cell.SCORING : Cell.AUDIENCE;
        if (alliance == Alliance.RED) redUpCell = up; else blueUpCell = up;
    }

    /** Robot heading (radians) that points the shooter at the goal. */
    public static double headingToGoal(Pose2D robot, Alliance alliance) {
        Pose2D goal = goal(robot, alliance);
        return MathFunc.angleWrap(Math.atan2(goal.y - robot.y, goal.x - robot.x) - SHOOTER_HEADING_OFFSET_RAD);
    }

    /** Straight-line distance (inches) from the robot's center to the goal. */
    public static double distanceToGoal(Pose2D robot, Alliance alliance) {
        return robot.distanceToPoint(goal(robot, alliance));
    }
}
