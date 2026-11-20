package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.LiftConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Lift on a PIDF this code runs itself (not the hub's RUN_TO_POSITION). All numbers are in
 * Constants.LiftConstants.
 *
 * Smooth moves: the PID doesn't chase the target directly but a "profiled" target that slides
 * toward it. Down: LANDING_SPEED + sqrt(2 * DOWN_DECEL * distance left), at most MAX_DOWN_SPEED,
 * so it comes down fast from high up and eases in at the end. Up: the same shape when
 * MAX_UP_SPEED_TPS > 0 (0 = jump straight to the target, the old behavior).
 *
 * Holding any position: the feedforward is the measured hold table at the profiled target
 * (HOLD_TABLE_*, from the Lift Hold Finder), or PIDF.f when the table is empty, and none in the
 * bottom zone. Within HOLD_DEADBAND_TICKS of a finished move it holds on the feedforward alone.
 *
 * The bottom: it rests unpowered instead of stalling against the stop. Once it has rested there
 * REZERO_REST_MS it re-zeroes the encoder, so the lift doesn't have to start all the way down.
 */
@Configurable
public class PIDControlledLift extends Lift {
    /** Slower than this (ticks/s) counts as standing still, for re-zeroing at the bottom. */
    private static final double STILL_TPS = 20;

    private final Telemetry telemetry;
    private final PIDFController controller;
    private PIDFCoefficients coefficients;
    private final int low, high;
    private int targetPosition = 0;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    private double profiledTarget = 0;
    private boolean descending = false, ascending = false;
    private final ElapsedTime profileTimer = new ElapsedTime();
    private final ElapsedTime restTimer = new ElapsedTime();
    private boolean resting = false, rezeroed = false;
    private boolean limp = false;

    /** False = ignore the hold table and use PIDF.f everywhere (the PIDF auto tuner measures F). */
    public boolean useHoldTable = true;
    /** Last power sent, and whether it was holding on the feedforward alone. */
    public double power = 0;
    public boolean holding = false;

    public PIDControlledLift(HardwareMap hardwareMap, Telemetry telemetry, int tolerance,
                             int highestPosition, int lowestPosition, boolean reversed,
                             PIDFCoefficients pidf) {
        super(hardwareMap, telemetry, tolerance, highestPosition, lowestPosition, reversed, pidf);
        this.telemetry = telemetry;
        this.low = lowestPosition;
        this.high = highestPosition;
        this.coefficients = pidf;

        controller = new PIDFController(pidf.p, pidf.i, pidf.d, pidf.f);

        motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = (int) getPosition();
        profiledTarget = targetPosition;
    }

    @Override
    public void setTargetPosition(int position) {
        // No controller reset here: D is on measurement, so target changes don't kick,
        // and resetting every loop while a button is held would wipe I and D.
        targetPosition = Range.clip(position, low, high);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    /** At the target (within TOLERANCE_TICKS) and the move has finished. */
    public boolean atTarget() {
        return profiledTarget == targetPosition && Math.abs(getPosition() - targetPosition) <= TOLERANCE_TICKS;
    }

    @Override
    public void liftUp()   { setTargetPosition(targetPosition + modifier); }
    @Override
    public void liftDown() { setTargetPosition(targetPosition - modifier); }

    @Override
    public void update() {
        // Pick up gains edited in place (e.g. from Panels) without needing setPidf()
        controller.setCoefficients(coefficients.p, coefficients.i, coefficients.d, coefficients.f);
        controller.setOutputLimits(-DOWN_POWER, UP_POWER); // gentle down, full up

        double current = motorEx.getCurrentPosition();
        double velocity = motorEx.getVelocity(); // ticks/s, negative = going down

        double dt = Math.min(profileTimer.seconds(), 0.1); // cap so a slow loop can't jump the target
        profileTimer.reset();

        if (limp) {
            // Hand mode: unpowered; the target follows the lift so it holds wherever it was left.
            targetPosition = (int) Range.clip(current, low, high);
            profiledTarget = targetPosition;
            setPower(0);
            return;
        }

        stepProfile(current, dt);

        double ff = holdPower(profiledTarget);
        double error = profiledTarget - current;
        holding = HOLD_DEADBAND_TICKS > 0 && profiledTarget == targetPosition
                && Math.abs(error) < HOLD_DEADBAND_TICKS;
        double out;
        if (holding) {
            controller.reset(); // fresh I and D once it has to correct again
            out = Range.clip(ff, -DOWN_POWER, UP_POWER);
        } else {
            double ks = profiledTarget > BOTTOM_ZONE_TICKS ? Math.signum(error) * KS : 0;
            out = controller.calculate(profiledTarget, current, ff + ks);
        }

        // Rest at the bottom instead of stalling against the stop, once it has eased in and slowed down
        boolean atBottom = targetPosition <= BOTTOM_ZONE_TICKS && profiledTarget <= BOTTOM_ZONE_TICKS
                && current <= BOTTOM_ZONE_TICKS + 5 && -velocity <= LANDING_SPEED_TPS;
        if (atBottom) out = 0;
        setPower(out);

        rezeroIfResting(atBottom, current, velocity);
        panelsTelemetry.getTelemetry().addData("Error", targetPosition - current);
    }

    private void setPower(double power) {
        this.power = power;
        motorEx.setPower(power);
    }

    /** Slide the profiled target toward the real one (see the class comment). */
    private void stepProfile(double current, double dt) {
        if (targetPosition < profiledTarget) {
            ascending = false;
            // Going down: start from where the lift actually is, then slide the target down smoothly
            if (!descending) {
                profiledTarget = Math.min(profiledTarget, current);
                descending = true;
            }
            profiledTarget = Math.max(targetPosition, profiledTarget - allowedDownSpeed(profiledTarget) * dt);
            if (profiledTarget <= targetPosition) descending = false;
        } else if (targetPosition > profiledTarget) {
            descending = false;
            if (MAX_UP_SPEED_TPS <= 0) {
                profiledTarget = targetPosition; // no limit going up
                return;
            }
            if (!ascending) {
                profiledTarget = Math.min(targetPosition, Math.max(profiledTarget, current));
                ascending = true;
            }
            profiledTarget = Math.min(targetPosition, profiledTarget + allowedUpSpeed(profiledTarget) * dt);
            if (profiledTarget >= targetPosition) ascending = false;
        } else {
            descending = false;
            ascending = false;
        }
    }

    /** How fast the profiled target may move down with this much distance left to go. */
    public double allowedDownSpeed(double profiled) {
        double distanceLeft = Math.max(0, profiled - targetPosition);
        return Math.min(MAX_DOWN_SPEED_TPS, LANDING_SPEED_TPS + Math.sqrt(2 * DOWN_DECEL_TPS2 * distanceLeft));
    }

    /** How fast the profiled target may move up with this much distance left to go. */
    public double allowedUpSpeed(double profiled) {
        double distanceLeft = Math.max(0, targetPosition - profiled);
        return Math.min(MAX_UP_SPEED_TPS, UP_MIN_SPEED_TPS + Math.sqrt(2 * UP_DECEL_TPS2 * distanceLeft));
    }

    /**
     * Power that holds the lift still at this position: the hold table (straight lines between
     * its points, flat past the ends), or PIDF.f if it has fewer than 2 points. 0 at the bottom.
     */
    public double holdPower(double ticks) {
        if (ticks <= BOTTOM_ZONE_TICKS) return 0;
        int n = Math.min(HOLD_TABLE_TICKS.length, HOLD_TABLE_POWER.length);
        if (!useHoldTable || n < 2) return coefficients.f;
        if (ticks <= HOLD_TABLE_TICKS[0]) return HOLD_TABLE_POWER[0];
        for (int i = 1; i < n; i++) {
            if (ticks <= HOLD_TABLE_TICKS[i]) {
                double t = (ticks - HOLD_TABLE_TICKS[i - 1]) / (HOLD_TABLE_TICKS[i] - HOLD_TABLE_TICKS[i - 1]);
                return HOLD_TABLE_POWER[i - 1] + t * (HOLD_TABLE_POWER[i] - HOLD_TABLE_POWER[i - 1]);
            }
        }
        return HOLD_TABLE_POWER[n - 1];
    }

    /** Once it has sat still at the bottom for REZERO_REST_MS, call that position 0 (once per rest). */
    private void rezeroIfResting(boolean atBottom, double current, double velocity) {
        if (targetPosition > BOTTOM_ZONE_TICKS) rezeroed = false;
        if (!atBottom || Math.abs(velocity) > STILL_TPS) {
            resting = false;
            return;
        }
        if (!resting) {
            resting = true;
            restTimer.reset();
        }
        if (REZERO_AT_BOTTOM && !rezeroed && current != 0 && restTimer.milliseconds() > REZERO_REST_MS) {
            resetEncoder();
            rezeroed = true;
        }
    }

    /** Limp = motor off and coasting, so it can be pushed by hand while its position keeps reading. */
    public void setLimp(boolean limp) {
        if (limp == this.limp) return;
        this.limp = limp;
        motorEx.setZeroPowerBehavior(limp ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        controller.reset();
    }

    public boolean isLimp() {
        return limp;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("lift (ticks)", "%.0f -> %d  (profile %.0f)  power %.2f%s%s", getPosition(),
                targetPosition, profiledTarget, power, holding ? "  holding" : "", limp ? "  LIMP" : "");
    }

    @Override
    public void setPidf(PIDFCoefficients pidf) {
        coefficients = pidf;
        controller.setCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
    }

    @Override
    public PIDFCoefficients pidf() { return coefficients; }

    @Override
    public void resetEncoder() {
        motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Encoder is now 0, so hold here instead of driving back to the old target
        targetPosition = Range.clip(0, low, high);
        profiledTarget = targetPosition;
        descending = false;
        ascending = false;
        controller.reset();
    }
}
