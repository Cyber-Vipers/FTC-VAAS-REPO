package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class PIDControlledLift extends Lift {
    private final Telemetry telemetry;
    private final PIDFController controller;
    private PIDFCoefficients coefficients;
    private final int low, high;
    private int targetPosition = 0;

    private static final int BOTTOM_ZONE = 20;

    // Soft landing (protects the bottom). Going up is never limited.
    // When the target is below the lift, the PID doesn't chase it directly. Instead it follows a
    // "profiled" target that slides down toward it at:
    //   LANDING_SPEED + sqrt(2 * DOWN_DECEL * distance left), at most MAX_DOWN_SPEED
    // so it comes down fast from high up and eases in at LANDING_SPEED at the end. The lift just
    // follows a moving target, so nothing pushes it back up and it doesn't bounce.
    public static double MAX_DOWN_SPEED = 1500; // ticks/s, high up
    public static double LANDING_SPEED = 150;   // ticks/s, at the end of the move down
    public static double DOWN_DECEL = 2500;     // ticks/s^2, how quickly it eases off near the end

    private double profiledTarget = 0;
    private boolean descending = false;
    private final ElapsedTime profileTimer = new ElapsedTime();

    public PIDControlledLift(HardwareMap hardwareMap, Telemetry telemetry, int tolerance,
                             int highestPosition, int lowestPosition, boolean reversed,
                             PIDFCoefficients pidf) {
        super(hardwareMap, telemetry, tolerance, highestPosition, lowestPosition, reversed, pidf);
        this.telemetry = telemetry;
        this.low = lowestPosition;
        this.high = highestPosition;
        this.coefficients = pidf;

        controller = new PIDFController(pidf.p, pidf.i, pidf.d, pidf.f);
        controller.setOutputLimits(-slowedPower, defaultPower); // gentle down, full up

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

    @Override
    public void liftUp()   { setTargetPosition(targetPosition + modifier); }
    @Override
    public void liftDown() { setTargetPosition(targetPosition - modifier); }

    @Override
    public void update() {
        // Pick up gains edited in place (e.g. from Panels) without needing setPidf()
        controller.setCoefficients(coefficients.p, coefficients.i, coefficients.d, coefficients.f);

        double current = motorEx.getCurrentPosition();
        double velocity = motorEx.getVelocity(); // ticks/s, negative = going down

        double dt = Math.min(profileTimer.seconds(), 0.1); // cap so a slow loop can't jump the target
        profileTimer.reset();

        if (targetPosition >= profiledTarget) {
            // Going up (or holding): no limit
            profiledTarget = targetPosition;
            descending = false;
        } else {
            // Going down: start from where the lift actually is, then slide the target down smoothly
            if (!descending) {
                profiledTarget = Math.min(profiledTarget, current);
                descending = true;
            }
            profiledTarget = Math.max(targetPosition, profiledTarget - allowedDownSpeed(profiledTarget) * dt);
            if (profiledTarget <= targetPosition) descending = false;
        }

        double ff = profiledTarget > BOTTOM_ZONE ? coefficients.f : 0;
        double power = controller.calculate(profiledTarget, current, ff);

        // Rest at the bottom instead of stalling against the stop, once it has eased in and slowed down
        if (targetPosition <= BOTTOM_ZONE && profiledTarget <= BOTTOM_ZONE
                && current <= BOTTOM_ZONE + 5 && -velocity <= LANDING_SPEED) {
            power = 0;
        }

        motorEx.setPower(power);

        telemetry.addData("Lift current", current);
        telemetry.addData("Lift target", targetPosition);
        telemetry.addData("Lift profiled target", "%.0f", profiledTarget);
        telemetry.addData("Lift power", power);
        telemetry.addData("Lift velocity", "%.0f", velocity);
    }

    /** How fast the profiled target may move down with this much distance left to go. */
    public double allowedDownSpeed(double profiled) {
        double distanceLeft = Math.max(0, profiled - targetPosition);
        return Math.min(MAX_DOWN_SPEED, LANDING_SPEED + Math.sqrt(2 * DOWN_DECEL * distanceLeft));
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
        controller.reset();
    }
}