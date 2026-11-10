package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

    // Soft landing (protects the plastic bottom). Going up is never limited.
    // While moving down, speed is capped so the lift can always brake before the bottom:
    //   allowed down speed = LANDING_SPEED + sqrt(2 * DOWN_DECEL * height above bottom), max MAX_DOWN_SPEED
    // so it can drop fast from high up but is down to LANDING_SPEED when it touches.
    public static double MAX_DOWN_SPEED = 1500; // ticks/s, high up
    public static double LANDING_SPEED = 150;   // ticks/s, when it reaches the bottom
    public static double DOWN_DECEL = 2500;     // ticks/s^2, how hard it may brake on the way down
    public static double BRAKE_GAIN = 0.002;    // upward power per tick/s over the limit

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

        double ff = targetPosition > BOTTOM_ZONE ? coefficients.f : 0;
        double power = controller.calculate(targetPosition, current, ff);

        // Soft landing: if it's coming down faster than allowed for this height, push up to brake.
        // This also catches the lift falling under gravity, not just being driven down.
        double allowedDown = allowedDownSpeed(current);
        double overspeed = -velocity - allowedDown;
        boolean braking = overspeed > 0;
        if (braking) {
            power = Math.max(power, coefficients.f + BRAKE_GAIN * overspeed);
            power = Math.min(power, defaultPower);
        }

        // Rest at the bottom instead of stalling against the stop, but only once it's slow
        // enough - cutting power while still fast would let it drop onto the plastic.
        if (targetPosition <= BOTTOM_ZONE && current <= BOTTOM_ZONE + 5 && -velocity <= LANDING_SPEED) {
            power = 0;
        }

        motorEx.setPower(power);

        telemetry.addData("Lift current", current);
        telemetry.addData("Lift target", targetPosition);
        telemetry.addData("Lift power", power);
        telemetry.addData("Lift velocity", "%.0f (down limit %.0f)%s", velocity, allowedDown, braking ? " BRAKING" : "");
    }

    /** Fastest the lift may move down at this height and still brake to LANDING_SPEED at the bottom. */
    public double allowedDownSpeed(double position) {
        double height = Math.max(0, position - low);
        return Math.min(MAX_DOWN_SPEED, LANDING_SPEED + Math.sqrt(2 * DOWN_DECEL * height));
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
        controller.reset();
    }
}