package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.utility;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.teleop.LiftShooterTeleop;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.PIDControlledLift;

import java.util.ArrayList;
import java.util.List;

/*
 * Automatic PIDF tuner for the HoneyComb PID lift (PIDControlledLift, not the plain Lift).
 *
 * The lift only ever moves between the bottom (bottomOut) and notable position 0, the
 * shooting position from LiftShooter (LiftShooter.shootingPosition). It never goes to the top.
 *
 * What it does (about 2 minutes, start with the lift all the way down):
 *   1. F: holds the shooting position with a small P only, measures how far it sags, and
 *         works out the power needed to hold the lift against gravity (F = P * sag).
 *   2. P: tries several P values, each one moving bottom -> shooting position, and scores
 *         how fast it gets there, how much it overshoots and how steady it holds.
 *   3. D: same test for several D values with the chosen P.
 *   4. Done: shows the numbers. A moves the lift between bottom and shooting position to check.
 *
 * I stays 0 - F holds the lift up, so I isn't needed.
 * Copy the result into LiftShooter's "liftPidf = new PIDFCoefficients(P, 0, D, F)".
 */
@Utility(name = "Honeycomb Lift PIDF Auto Tuner")
public class LiftPIDFAutoTuner extends LinearOpMode {
    // Same lift setup as LiftShooter
    private static final int HIGHEST_LIFT = 1062;
    private static final int LOWEST_LIFT = 0;
    private static final int SHOOTING_POSITION_INDEX = 0;

    private static final double START_P = 0.003;
    private static final double[] P_CANDIDATES = {0.002, 0.004, 0.006, 0.01, 0.015, 0.02, 0.03};
    private static final double[] D_MULTIPLIERS_OF_P = {0, 0.01, 0.02, 0.05, 0.1}; // seconds
    private static final int REPEATS = 2;

    private static final double MOVE_RECORD_S = 2.0;
    private static final double STEADY_WINDOW_S = 0.5;
    private static final double BOTTOM_WAIT_MAX_S = 3.0;
    private static final double BOTTOM_REST_S = 0.5;
    private static final double F_HOLD_S = 2.5;
    private static final int SETTLE_BAND = 10;      // ticks from target that count as "there"
    private static final int OVERSHOOT_ABORT = 150; // ticks past target -> stop this test early

    private PIDControlledLift lift;
    // Edited in place; PIDControlledLift.update() reloads it every loop
    private final PIDFCoefficients gains = new PIDFCoefficients(START_P, 0, 0, 0);
    private int target;
    // Fastest the lift has come down in the last few ticks before the bottom (soft-landing check)
    private static final int LANDING_CHECK_HEIGHT = 40;
    private double maxLandingSpeed = 0;

    private String phase = "";
    private final List<String> results = new ArrayList<>();

    private static class MoveResult {
        double cost, iae, overshoot, settleS, steadyErr, wobble;
    }

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        lift = new PIDControlledLift(hardwareMap, telemetry, LiftShooterTeleop.globalTolerance,
                HIGHEST_LIFT, LOWEST_LIFT, false, gains);
        lift.resetEncoder();
        lift.saveNotablePosition(LiftShooterTeleop.shootingPosition);
        target = lift.notablePositions.get(SHOOTING_POSITION_INDEX);

        while (opModeInInit()) {
            telemetry.addLine("Lift PIDF auto tuner");
            telemetry.addLine("Make sure the lift is ALL THE WAY DOWN and clear before pressing start.");
            telemetry.addLine();
            telemetry.addData("Moves between", "bottom and notable position 0 (%d ticks)", target);
            telemetry.addData("Lift position", lift.getPosition());
            telemetry.update();
        }
        if (isStopRequested()) return;

        // 1. Feedforward: with P only, the lift sags until P * sag = power needed to hold it
        phase = "F: measuring hold power at shooting position";
        setGains(START_P, 0, 0);
        if (!goToBottom()) return;
        lift.goToNotablePosition(SHOOTING_POSITION_INDEX);
        if (!runFor(F_HOLD_S - STEADY_WINDOW_S)) return;
        double sag = averageError(STEADY_WINDOW_S);
        if (!opModeIsActive()) return;
        if (lift.getPosition() < target / 2.0) {
            fail(String.format("Lift only reached %.0f of %d ticks with P = %.3f. "
                    + "Check the lift motor direction (the 'reversed' flag) and that the encoder works.",
                    lift.getPosition(), target, START_P));
            return;
        }
        double f = Math.max(0, START_P * sag);
        results.add(String.format("Sag %.0f ticks -> F = %.3f", sag, f));

        // 2. P sweep
        double bestP = START_P;
        double bestCost = Double.MAX_VALUE;
        for (double p : P_CANDIDATES) {
            MoveResult r = testGains(p, 0, f, "P sweep");
            if (r == null) return;
            if (r.cost < bestCost) {
                bestCost = r.cost;
                bestP = p;
            }
        }
        results.add(String.format("Best P = %.4f", bestP));

        // 3. D sweep
        double bestD = 0;
        bestCost = Double.MAX_VALUE;
        for (double mult : D_MULTIPLIERS_OF_P) {
            double d = bestP * mult;
            MoveResult r = testGains(bestP, d, f, "D sweep");
            if (r == null) return;
            if (r.cost < bestCost) {
                bestCost = r.cost;
                bestD = d;
            }
        }
        results.add(String.format("Best D = %.5f", bestD));

        // 4. Final check
        MoveResult fin = testGains(bestP, bestD, f, "Final check");
        if (fin == null) return;

        String code = String.format("new PIDFCoefficients(%.4f, 0, %.5f, %.3f)", bestP, bestD, f);
        RobotLog.ii("LiftPIDFAutoTuner", "Result: %s", code);

        boolean atShooting = true;
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                atShooting = !atShooting;
                if (atShooting) {
                    lift.goToNotablePosition(SHOOTING_POSITION_INDEX);
                } else {
                    lift.bottomOut();
                }
            }
            lift.update();

            telemetry.addLine("=== DONE ===");
            telemetry.addData("P", "%.4f", bestP);
            telemetry.addData("I", "0");
            telemetry.addData("D", "%.5f", bestD);
            telemetry.addData("F", "%.3f", f);
            telemetry.addLine(code);
            telemetry.addLine();
            telemetry.addData("Time to reach", "%.2f s, overshoot %.0f ticks", fin.settleS, fin.overshoot);
            telemetry.addData("Hold error / wobble", "%.1f / %.1f ticks", fin.steadyErr, fin.wobble);
            telemetry.addData("Fastest landing", "%.0f ticks/s (limit at bottom %.0f)",
                    maxLandingSpeed, PIDControlledLift.LANDING_SPEED);
            telemetry.addLine();
            telemetry.addData("A: move to", atShooting ? "bottom" : "shooting position");
            telemetry.update();
        }
        lift.bottomOut();
        lift.motorEx.setPower(0);
    }

    private void setGains(double p, double d, double f) {
        gains.p = p;
        gains.i = 0;
        gains.d = d;
        gains.f = f;
    }

    /** Runs REPEATS bottom -> shooting position moves with these gains and averages them. Null if stopped. */
    private MoveResult testGains(double p, double d, double f, String name) {
        setGains(p, d, f);
        MoveResult avg = new MoveResult();
        for (int n = 0; n < REPEATS; n++) {
            phase = String.format("%s: P %.4f  D %.5f  (run %d/%d)", name, p, d, n + 1, REPEATS);
            MoveResult r = moveTest();
            if (r == null) return null;
            avg.cost += r.cost / REPEATS;
            avg.iae += r.iae / REPEATS;
            avg.overshoot += r.overshoot / REPEATS;
            avg.settleS += r.settleS / REPEATS;
            avg.steadyErr += r.steadyErr / REPEATS;
            avg.wobble += r.wobble / REPEATS;
        }
        results.add(String.format("%s P %.4f D %.5f: %.2fs, os %.0f, wob %.1f -> %.3f",
                name, p, d, avg.settleS, avg.overshoot, avg.wobble, avg.cost));
        return avg;
    }

    /** Lowers the lift, moves it to notable position 0 and scores the move. Null if stopped. */
    private MoveResult moveTest() {
        if (!goToBottom()) return null;

        lift.goToNotablePosition(SHOOTING_POSITION_INDEX);
        ElapsedTime timer = new ElapsedTime();
        double last = 0, iae = 0, peak = 0, lastOutside = 0;
        double steadySum = 0, steadySumSq = 0;
        int steadyCount = 0;
        boolean aborted = false;
        while (opModeIsActive() && timer.seconds() < MOVE_RECORD_S) {
            lift.update();
            double now = timer.seconds();
            double pos = lift.getPosition();
            double err = target - pos;
            iae += Math.abs(err) * (now - last);
            last = now;
            peak = Math.max(peak, pos);
            if (Math.abs(err) > SETTLE_BAND) lastOutside = now;
            if (now > MOVE_RECORD_S - STEADY_WINDOW_S) {
                steadySum += pos;
                steadySumSq += pos * pos;
                steadyCount++;
            }
            if (pos > target + OVERSHOOT_ABORT) {
                aborted = true;
                break;
            }
            showStatus();
        }
        if (!opModeIsActive()) return null;

        MoveResult r = new MoveResult();
        if (aborted || steadyCount == 0) {
            lift.bottomOut();
            r.overshoot = peak - target;
            r.settleS = MOVE_RECORD_S;
            r.cost = 1000; // way too aggressive
            return r;
        }
        double mean = steadySum / steadyCount;
        r.iae = iae / target;
        r.overshoot = Math.max(0, peak - target);
        r.settleS = lastOutside;
        r.steadyErr = Math.abs(target - mean);
        r.wobble = Math.sqrt(Math.max(0, steadySumSq / steadyCount - mean * mean));
        // Lower is better: fast, little overshoot, holds steady and on target
        r.cost = r.iae + 0.01 * r.overshoot + 0.05 * r.wobble + 0.01 * r.steadyErr;
        return r;
    }

    /** Sends the lift to the bottom and waits for it to get there and stop. */
    private boolean goToBottom() {
        lift.bottomOut();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime resting = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < BOTTOM_WAIT_MAX_S) {
            lift.update();
            if (lift.getPosition() < LOWEST_LIFT + LANDING_CHECK_HEIGHT) {
                maxLandingSpeed = Math.max(maxLandingSpeed, -lift.motorEx.getVelocity());
            }
            if (lift.getPosition() > LOWEST_LIFT + 30) resting.reset();
            if (resting.seconds() > BOTTOM_REST_S) break;
            showStatus();
        }
        return opModeIsActive();
    }

    private boolean runFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            lift.update();
            showStatus();
        }
        return opModeIsActive();
    }

    private double averageError(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        double sum = 0;
        int count = 0;
        while (opModeIsActive() && timer.seconds() < seconds) {
            lift.update();
            sum += target - lift.getPosition();
            count++;
            showStatus();
        }
        return count == 0 ? 0 : sum / count;
    }

    private void showStatus() {
        telemetry.addData("Phase", phase);
        telemetry.addData("Gains", "P %.4f  D %.5f  F %.3f", gains.p, gains.d, gains.f);
        telemetry.addData("Fastest landing so far", "%.0f ticks/s", maxLandingSpeed);
        telemetry.addLine();
        int start = Math.max(0, results.size() - 10);
        for (int n = start; n < results.size(); n++) {
            telemetry.addLine(results.get(n));
        }
        telemetry.update();
    }

    private void fail(String message) {
        lift.bottomOut();
        RobotLog.ee("LiftPIDFAutoTuner", message);
        while (opModeIsActive()) {
            lift.update();
            telemetry.addLine("TUNING STOPPED");
            telemetry.addLine(message);
            telemetry.update();
        }
        lift.motorEx.setPower(0);
    }
}
