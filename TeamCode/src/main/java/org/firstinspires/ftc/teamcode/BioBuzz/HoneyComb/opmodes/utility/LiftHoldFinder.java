package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.utility;

import static org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.LiftConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.PIDControlledLift;

import java.util.Locale;

/**
 * Measures the lift's hold power (the motor power that keeps it still) at each
 * LiftConstants.HOLD_FINDER_TICKS position, for the hold table. A lift that pivots needs a
 * different hold power at every position, so one F only holds well near where it was tuned.
 *
 * At each position it settles there with the PID, then raises the power slowly (no PID) until
 * the lift creeps up HOLD_FINDER_MOVE_TICKS, settles again, and lowers the power until it sags
 * that far. Hold power = the middle of the two; half the gap is friction (-> KS).
 *
 * Start with the lift all the way down and clear. B stops at any time. When it's done:
 *   - the new table is used for the rest of this run: dpad up/down moves between the measured
 *     positions so you can check it holds still at each one (with the flywheel running too);
 *   - copy the printed lines (also in the Driver Station log) into Constants.LiftConstants.
 */
@TeleOp(name = "HoneyComb Lift Hold Finder", group = "Honeycomb Tuning")
public class LiftHoldFinder extends LinearOpMode {
    private static final double SETTLE_S = 1.5, SETTLE_MAX_S = 4, RAMP_MAX_S = 10;
    /** The ramp never goes more than this far from the power that was holding the lift. */
    private static final double RAMP_LIMIT = 0.4;

    private PIDControlledLift lift;
    private String stopReason = "stopped with B";

    @Override
    public void runOpMode() {
        lift = new PIDControlledLift(hardwareMap, telemetry, TOLERANCE_TICKS, HIGHEST_TICKS, LOWEST_TICKS,
                REVERSED, PIDF);
        lift.useHoldTable = false; // settle on plain PIDF while measuring
        lift.resetEncoder();

        while (opModeInInit()) {
            telemetry.addLine("Lift Hold Finder: the lift must be ALL THE WAY DOWN and clear.");
            telemetry.addData("positions", java.util.Arrays.toString(HOLD_FINDER_TICKS));
            telemetry.addData("lift", "%.0f ticks", lift.getPosition());
            telemetry.addLine("Start to begin, B to stop at any time.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        int n = HOLD_FINDER_TICKS.length;
        double[] at = new double[n], creep = new double[n], sag = new double[n];
        boolean ok = true;
        for (int i = 0; i < n && ok; i++) {
            int target = (int) HOLD_FINDER_TICKS[i];
            String where = String.format(Locale.US, "position %d/%d (%d ticks)", i + 1, n, target);
            ok = settle(target, where);
            if (ok) {
                at[i] = lift.getPosition();
                creep[i] = ramp(+1, where);
                ok = !Double.isNaN(creep[i]);
            }
            if (ok) ok = settle(target, where);
            if (ok) {
                sag[i] = ramp(-1, where);
                ok = !Double.isNaN(sag[i]);
            }
        }
        if (!ok) {
            lift.bottomOut();
            while (opModeIsActive()) {
                lift.update();
                telemetry.addLine("Stopped before finishing -- nothing saved. Lift going down.");
                telemetry.addData("why", stopReason);
                telemetry.update();
            }
            return;
        }

        // Results: hold = middle of creep and sag at each position, friction = half the gap.
        double[] hold = new double[n];
        double friction = 0;
        for (int i = 0; i < n; i++) {
            hold[i] = (creep[i] + sag[i]) / 2;
            friction += (creep[i] - sag[i]) / 2 / n;
        }
        HOLD_TABLE_TICKS = at;
        HOLD_TABLE_POWER = hold;
        KS = Math.max(0, friction);
        lift.useHoldTable = true;
        String ticksLine = "HOLD_TABLE_TICKS = {" + join(at, "%.0f") + "};";
        String powerLine = "HOLD_TABLE_POWER = {" + join(hold, "%.3f") + "};";
        String ksLine = String.format(Locale.US, "KS = %.3f;", KS);
        telemetry.log().add("Paste into Constants.LiftConstants:");
        telemetry.log().add(ticksLine);
        telemetry.log().add(powerLine);
        telemetry.log().add(ksLine);
        RobotLog.ii("LiftHoldFinder", "%s %s %s", ticksLine, powerLine, ksLine);

        // Check: move between the measured positions with the new table.
        int index = n / 2;
        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) index = Math.min(n - 1, index + 1);
            if (gamepad1.dpadDownWasPressed()) index = Math.max(0, index - 1);
            boolean down = gamepad1.b;
            if (down) lift.bottomOut(); else lift.setTargetPosition((int) at[index]);
            lift.update();

            telemetry.addLine("=== DONE: paste into Constants.LiftConstants ===");
            telemetry.addLine(ticksLine);
            telemetry.addLine(powerLine);
            telemetry.addLine(ksLine);
            telemetry.addLine();
            for (int i = 0; i < n; i++) {
                telemetry.addLine(String.format(Locale.US, "  %.0f ticks: creeps up at %.3f, sags at %.3f -> hold %.3f",
                        at[i], creep[i], sag[i], hold[i]));
            }
            telemetry.addLine();
            telemetry.addData("checking", down ? "bottom (B held)" : String.format(Locale.US, "%.0f ticks", at[index]));
            lift.addTelemetry(telemetry);
            telemetry.addLine("dpad up/down = next position, hold B = lift down. It should hold still at each one.");
            telemetry.update();
        }
    }

    /**
     * Hold the target with the PID until it has been still there SETTLE_S. False if stopped, or if
     * it never got there (jammed, encoder unplugged, or not all the way down at Init) -- measuring
     * from a stall would push the lift against a stop.
     */
    private boolean settle(int target, String where) {
        lift.setTargetPosition(target);
        ElapsedTime total = new ElapsedTime(), there = new ElapsedTime();
        while (opModeIsActive() && !gamepad1.b && total.seconds() < SETTLE_MAX_S) {
            lift.update();
            if (!lift.atTarget()) there.reset();
            if (there.seconds() > SETTLE_S) return true;
            status(where, "settling", lift.power);
        }
        if (opModeIsActive() && !gamepad1.b) {
            stopReason = String.format(Locale.US, "couldn't reach %d ticks (got to %.0f): check the lift was all the "
                    + "way down at Init, its encoder cable, and HOLD_FINDER_TICKS", target, lift.getPosition());
        }
        return false;
    }

    /**
     * Ramp the power from the PID's holding power, up (direction +1) or down (-1), with no PID,
     * until the lift moves HOLD_FINDER_MOVE_TICKS that way. Returns that power, or NaN if stopped
     * or it never moved.
     */
    private double ramp(int direction, String where) {
        double startPower = lift.power;
        double power = startPower;
        double start = lift.getPosition();
        ElapsedTime timer = new ElapsedTime(), loop = new ElapsedTime();
        while (opModeIsActive() && !gamepad1.b && timer.seconds() < RAMP_MAX_S) {
            power += direction * HOLD_FINDER_RAMP * loop.seconds();
            loop.reset();
            // Never more than RAMP_LIMIT away from where it started holding.
            power = Math.max(startPower - RAMP_LIMIT, Math.min(startPower + RAMP_LIMIT, power));
            power = Math.max(-1, Math.min(1, power));
            lift.motorEx.setPower(power);
            double moved = lift.getPosition() - start;
            if (direction * moved > HOLD_FINDER_MOVE_TICKS) return power;
            status(where, direction > 0 ? "raising power until it creeps up" : "lowering power until it sags", power);
        }
        lift.motorEx.setPower(startPower);
        if (opModeIsActive() && !gamepad1.b) {
            stopReason = String.format(Locale.US, "the lift didn't %s within %.2f power of its hold -- check for a jam",
                    direction > 0 ? "move up" : "move down", RAMP_LIMIT);
        }
        return Double.NaN;
    }

    private void status(String where, String phase, double power) {
        telemetry.addData("measuring", where);
        telemetry.addData("phase", phase);
        telemetry.addData("power", "%.3f", power);
        telemetry.addData("lift", "%.0f ticks", lift.getPosition());
        telemetry.addLine("B = stop");
        telemetry.update();
    }

    private static String join(double[] values, String format) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < values.length; i++) {
            if (i > 0) sb.append(", ");
            sb.append(String.format(Locale.US, format, values[i]));
        }
        return sb.toString();
    }
}
