package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

/*
 * Automatic velocity PIDF tuner for the HoneyComb shooter flywheel.
 *
 * Shooter: 96mm wheel, GT2 20T -> 20T pulleys (1:1) on a 164T belt, so the wheel turns at motor speed.
 *
 * What it does (about 2 minutes, flywheel spins up to full power - make sure nothing is loaded):
 *   1. Feedforward: runs the motor at a few fixed powers and measures speed to find F.
 *   2. P sweep:     tries several P values. Each test drops the speed to 60% of the target (like after
 *                   a shot) and measures how fast and cleanly it recovers. Keeps the best one.
 *   3. D sweep:     same test for several D values with the chosen P (I is always 0).
 *   4. Done:        shows the final numbers and holds the target so you can check it.
 *
 * Before start (init):
 *   dpad up/down - target wheel RPM (left bumper switches coarse/fine)
 *   X            - flip flywheel direction (tune in the direction you shoot)
 * After tuning:
 *   A            - flywheel on/off at the target
 *
 * The SDK resets PIDF every opmode, so copy the printed numbers into your code,
 * e.g. LiftShooter's "new PIDFCoefficients(P, 0, D, F)".
 */
@TeleOp(name = "Honeycomb Flywheel PIDF Auto Tuner", group = "Honeycomb")
public class FlywheelPIDFAutoTuner extends LinearOpMode {
    private static final String FLYWHEEL_NAME = "flywheel";

    private static final double WHEEL_DIAMETER_M = 0.096;
    private static final double WHEEL_REVS_PER_MOTOR_REV = 20.0 / 20.0; // GT2 20T to 20T
    // 0 = use ticks/rev from the motor type in the robot config. Set this if the config motor type is wrong.
    private static final double TICKS_PER_MOTOR_REV_OVERRIDE = 0;

    // The hub's velocity loop maps full power to 32767, so F = 32767 * power / velocity
    private static final double HUB_MAX_OUTPUT = 32767;
    private static final double[] FEEDFORWARD_POWERS = {0.4, 0.6, 0.8, 1.0};
    private static final double FEEDFORWARD_SETTLE_S = 2.0;
    private static final double FEEDFORWARD_SAMPLE_S = 1.0;
    private static final double MAX_TARGET_FRACTION = 0.9; // leave headroom so the loop can recover

    private static final double[] P_MULTIPLIERS_OF_F = {0.03, 0.06, 0.1, 0.2, 0.4, 0.7, 1.0};
    private static final double[] P_REFINE_MULTIPLIERS = {0.75, 1.33};
    private static final double[] D_MULTIPLIERS_OF_P = {0, 0.01, 0.03, 0.1, 0.3};
    private static final int REPEATS = 2;

    private static final double STEP_LOW_FRACTION = 0.6;
    private static final double STEP_SETTLE_S = 1.5;
    private static final double STEP_RECORD_S = 1.5;
    private static final double STEADY_WINDOW_S = 0.5;
    private static final double SETTLE_BAND = 0.02; // within 2% of target counts as settled

    private DcMotorEx flywheel;
    private double ticksPerWheelRev;
    private double targetRpm = 3000;
    private double targetTps;
    private boolean reversed = false;

    private String phase = "";
    private final List<String> results = new ArrayList<>();

    private static class StepResult {
        double cost, iae, overshootPct, settleS, steadyErrPct, wobblePct;
    }

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ticksPerMotorRev = TICKS_PER_MOTOR_REV_OVERRIDE > 0
                ? TICKS_PER_MOTOR_REV_OVERRIDE
                : flywheel.getMotorType().getTicksPerRev();
        ticksPerWheelRev = ticksPerMotorRev / WHEEL_REVS_PER_MOTOR_REV;

        boolean fine = false;
        while (opModeInInit()) {
            if (gamepad1.leftBumperWasPressed()) fine = !fine;
            double step = fine ? 10 : 100;
            if (gamepad1.dpadUpWasPressed()) targetRpm += step;
            if (gamepad1.dpadDownWasPressed()) targetRpm = Math.max(step, targetRpm - step);
            if (gamepad1.xWasPressed()) reversed = !reversed;

            telemetry.addLine("FLYWHEEL WILL SPIN UP TO FULL POWER - UNLOAD ALL BALLS");
            telemetry.addLine();
            telemetry.addData("Target wheel RPM", "%.0f  (%s steps)", targetRpm, fine ? "fine" : "coarse");
            telemetry.addData("Target ticks/s", "%.0f", rpmToTps(targetRpm));
            telemetry.addData("Wheel surface speed", "%.1f m/s", surfaceSpeed(targetRpm));
            telemetry.addData("Direction", reversed ? "REVERSE" : "FORWARD");
            telemetry.addData("Motor type in config", flywheel.getMotorType().getName());
            telemetry.addData("Ticks per wheel rev", "%.1f", ticksPerWheelRev);
            telemetry.addLine();
            telemetry.addLine("dpad up/down: RPM | left bumper: fine/coarse | X: direction");
            telemetry.update();
        }
        if (isStopRequested()) return;

        flywheel.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        targetTps = rpmToTps(targetRpm);

        // 1. Feedforward
        double sumPV = 0, sumVV = 0, fullPowerTps = 0;
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        for (double power : FEEDFORWARD_POWERS) {
            phase = String.format("Feedforward: power %.1f", power);
            flywheel.setPower(power);
            if (!waitSeconds(FEEDFORWARD_SETTLE_S)) return;
            double v = averageVelocity(FEEDFORWARD_SAMPLE_S);
            if (!opModeIsActive()) return;
            results.add(String.format("power %.1f -> %.0f ticks/s (%.0f RPM)", power, v, tpsToRpm(v)));
            sumPV += power * v;
            sumVV += v * v;
            fullPowerTps = v;
        }
        flywheel.setPower(0);

        if (Math.abs(fullPowerTps) < 50) {
            fail("Encoder reads ~0 at full power. Check the flywheel encoder cable is in the same port as its power cable.");
            return;
        }
        if (fullPowerTps < 0) {
            fail("Encoder counts backwards compared to the motor. The velocity loop can't work like this - check wiring.");
            return;
        }
        double f = HUB_MAX_OUTPUT * sumPV / sumVV;
        results.add(String.format("F = %.3f  (max %.0f RPM)", f, tpsToRpm(fullPowerTps)));

        if (targetTps > MAX_TARGET_FRACTION * fullPowerTps) {
            targetTps = MAX_TARGET_FRACTION * fullPowerTps;
            targetRpm = tpsToRpm(targetTps);
            results.add(String.format("Target too close to max, lowered to %.0f RPM", targetRpm));
        }

        // 2. P sweep
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double bestP = 0;
        double bestCost = Double.MAX_VALUE;
        for (double mult : P_MULTIPLIERS_OF_F) {
            double p = f * mult;
            StepResult r = testGains(p, 0, f, "P sweep");
            if (r == null) return;
            if (r.cost < bestCost) {
                bestCost = r.cost;
                bestP = p;
            }
        }
        double coarseP = bestP;
        for (double mult : P_REFINE_MULTIPLIERS) {
            double p = coarseP * mult;
            StepResult r = testGains(p, 0, f, "P refine");
            if (r == null) return;
            if (r.cost < bestCost) {
                bestCost = r.cost;
                bestP = p;
            }
        }
        results.add(String.format("Best P = %.3f", bestP));

        // 3. D sweep
        double bestD = 0;
        bestCost = Double.MAX_VALUE;
        for (double mult : D_MULTIPLIERS_OF_P) {
            double d = bestP * mult;
            StepResult r = testGains(bestP, d, f, "D sweep");
            if (r == null) return;
            if (r.cost < bestCost) {
                bestCost = r.cost;
                bestD = d;
            }
        }
        results.add(String.format("Best D = %.3f", bestD));

        // 4. Final check with the chosen gains
        StepResult fin = testGains(bestP, bestD, f, "Final check");
        if (fin == null) return;

        String code = String.format("new PIDFCoefficients(%.3f, 0, %.3f, %.3f)", bestP, bestD, f);
        RobotLog.ii("FlywheelPIDFAutoTuner", "Result for %.0f RPM: %s", targetRpm, code);

        boolean on = true;
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) on = !on;
            flywheel.setVelocity(on ? targetTps : 0);
            double v = flywheel.getVelocity();

            telemetry.addLine("=== DONE ===");
            telemetry.addData("P", "%.3f", bestP);
            telemetry.addData("I", "0");
            telemetry.addData("D", "%.3f", bestD);
            telemetry.addData("F", "%.3f", f);
            telemetry.addLine(code);
            telemetry.addLine();
            telemetry.addData("Recovery from 60%", "%.2f s, overshoot %.1f%%", fin.settleS, fin.overshootPct);
            telemetry.addData("Steady error / wobble", "%.1f%% / %.1f%%", fin.steadyErrPct, fin.wobblePct);
            telemetry.addLine();
            telemetry.addData("Flywheel (A to toggle)", on ? "ON" : "OFF");
            telemetry.addData("Target", "%.0f RPM (%.0f ticks/s)", targetRpm, targetTps);
            telemetry.addData("Actual", "%.0f RPM (%.0f ticks/s)", tpsToRpm(v), v);
            telemetry.update();
        }
        flywheel.setPower(0);
    }

    /** Runs REPEATS step tests with these gains and returns the averaged result, or null if stopped. */
    private StepResult testGains(double p, double d, double f, String name) {
        flywheel.setVelocityPIDFCoefficients(p, 0, d, f);
        StepResult avg = new StepResult();
        for (int n = 0; n < REPEATS; n++) {
            phase = String.format("%s: P %.3f  D %.3f  (run %d/%d)", name, p, d, n + 1, REPEATS);
            StepResult r = stepTest();
            if (r == null) return null;
            avg.cost += r.cost / REPEATS;
            avg.iae += r.iae / REPEATS;
            avg.overshootPct += r.overshootPct / REPEATS;
            avg.settleS += r.settleS / REPEATS;
            avg.steadyErrPct += r.steadyErrPct / REPEATS;
            avg.wobblePct += r.wobblePct / REPEATS;
        }
        results.add(String.format("%s P %.3f D %.3f: %.2fs, os %.0f%%, wob %.1f%% -> %.3f",
                name, p, d, avg.settleS, avg.overshootPct, avg.wobblePct, avg.cost));
        return avg;
    }

    /** Settles at 60% of target, steps up to the target and scores the recovery. Null if stopped. */
    private StepResult stepTest() {
        flywheel.setVelocity(targetTps * STEP_LOW_FRACTION);
        if (!waitSeconds(STEP_SETTLE_S)) return null;

        flywheel.setVelocity(targetTps);
        ElapsedTime timer = new ElapsedTime();
        double last = 0, iae = 0, peak = 0, lastOutside = 0;
        double steadySum = 0, steadySumSq = 0;
        int steadyCount = 0;
        while (opModeIsActive() && timer.seconds() < STEP_RECORD_S) {
            double now = timer.seconds();
            double v = flywheel.getVelocity();
            double err = targetTps - v;
            iae += Math.abs(err) * (now - last);
            last = now;
            peak = Math.max(peak, v);
            if (Math.abs(err) > SETTLE_BAND * targetTps) lastOutside = now;
            if (now > STEP_RECORD_S - STEADY_WINDOW_S) {
                steadySum += v;
                steadySumSq += v * v;
                steadyCount++;
            }
            showStatus(v);
        }
        if (!opModeIsActive() || steadyCount == 0) return null;

        double mean = steadySum / steadyCount;
        double std = Math.sqrt(Math.max(0, steadySumSq / steadyCount - mean * mean));

        StepResult r = new StepResult();
        r.iae = iae / targetTps;
        r.overshootPct = Math.max(0, (peak - targetTps) / targetTps * 100);
        r.settleS = lastOutside;
        r.steadyErrPct = Math.abs(mean - targetTps) / targetTps * 100;
        r.wobblePct = std / targetTps * 100;
        // Lower is better: fast recovery, little overshoot, steady and on target
        r.cost = r.iae + 0.01 * r.overshootPct + 0.05 * r.wobblePct + 0.02 * r.steadyErrPct;
        return r;
    }

    private boolean waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            showStatus(flywheel.getVelocity());
        }
        return opModeIsActive();
    }

    private double averageVelocity(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        double sum = 0;
        int count = 0;
        while (opModeIsActive() && timer.seconds() < seconds) {
            double v = flywheel.getVelocity();
            sum += v;
            count++;
            showStatus(v);
        }
        return count == 0 ? 0 : sum / count;
    }

    private void showStatus(double velocity) {
        telemetry.addData("Phase", phase);
        telemetry.addData("Velocity", "%.0f ticks/s (%.0f RPM)", velocity, tpsToRpm(velocity));
        telemetry.addLine();
        int start = Math.max(0, results.size() - 10);
        for (int n = start; n < results.size(); n++) {
            telemetry.addLine(results.get(n));
        }
        telemetry.update();
    }

    private void fail(String message) {
        flywheel.setPower(0);
        RobotLog.ee("FlywheelPIDFAutoTuner", message);
        while (opModeIsActive()) {
            telemetry.addLine("TUNING STOPPED");
            telemetry.addLine(message);
            telemetry.addLine();
            for (String line : results) telemetry.addLine(line);
            telemetry.update();
        }
    }

    private double rpmToTps(double rpm) {
        return rpm / 60.0 * ticksPerWheelRev;
    }

    private double tpsToRpm(double tps) {
        return tps / ticksPerWheelRev * 60.0;
    }

    private double surfaceSpeed(double rpm) {
        return rpm / 60.0 * Math.PI * WHEEL_DIAMETER_M;
    }
}
