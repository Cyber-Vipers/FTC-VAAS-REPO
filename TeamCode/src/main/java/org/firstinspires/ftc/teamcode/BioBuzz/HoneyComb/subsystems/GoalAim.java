package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.DriveConstants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.neuronpathing.MathFunc;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Turns the robot to a heading through MecanumDrive's turn override, so the driver keeps the
 * left stick while it aims. Gains are in Constants.DriveConstants, tolerances in ShooterConstants.
 *
 * turn = AIM_KP * error(deg) - AIM_KD * turn rate(deg/s), plus AIM_KS toward the target while
 * outside the tolerance. The D term uses the Pinpoint's measured turn rate, not the change in
 * error / loop time, which turns one noisy reading into a power spike when the loop is fast.
 */
public class GoalAim {
    private final MecanumDrive drive;
    private final NeuronLocalizer localizer;
    private final ElapsedTime aimTimer = new ElapsedTime();
    private boolean active = false;

    /** Heading error (degrees, + = needs to turn counter-clockwise) from the last update(). */
    public double errorDeg = 0;

    public GoalAim(MecanumDrive drive, NeuronLocalizer localizer) {
        this.drive = drive;
        this.localizer = localizer;
    }

    /**
     * Turn toward targetHeading (radians, field frame) this loop. True once within
     * AIM_TOLERANCE_DEG, or AIM_RELAXED_TOLERANCE_DEG after AIM_RELAX_MS of aiming.
     */
    public boolean update(Pose2D pose, double targetHeading) {
        if (!active) {
            active = true;
            aimTimer.reset();
        }
        errorDeg = Math.toDegrees(MathFunc.angleWrap(targetHeading - pose.h));
        double rateDeg = localizer.getAngularVelocityDegPerSec(); // from this loop's Pinpoint read
        double turn = AIM_KP * errorDeg - AIM_KD * rateDeg;
        if (Math.abs(errorDeg) > ShooterConstants.AIM_TOLERANCE_DEG) turn += Math.signum(errorDeg) * AIM_KS;
        turn = Range.clip(turn, -AIM_MAX_TURN, AIM_MAX_TURN);
        drive.setTurnOverride(AIM_TURN_REVERSED ? -turn : turn);

        double tolerance = aimTimer.milliseconds() > ShooterConstants.AIM_RELAX_MS
                ? ShooterConstants.AIM_RELAXED_TOLERANCE_DEG : ShooterConstants.AIM_TOLERANCE_DEG;
        return Math.abs(errorDeg) < tolerance;
    }

    /** Hand turning back to the driver. */
    public void stop() {
        active = false;
        drive.clearTurnOverride();
    }

    public boolean isActive() {
        return active;
    }
}
