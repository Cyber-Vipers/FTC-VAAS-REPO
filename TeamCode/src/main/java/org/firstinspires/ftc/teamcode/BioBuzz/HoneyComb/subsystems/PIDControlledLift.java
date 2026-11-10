package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDControlledLift extends Lift {
    private final Telemetry telemetry;
    private final PIDFController controller;
    private PIDFCoefficients coefficients;
    private final int low, high;
    private int targetPosition = 0;

    private static final int BOTTOM_ZONE = 20;

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
        position = Range.clip(position, low, high);
        if (position != targetPosition) controller.reset();
        targetPosition = position;
    }

    @Override
    public void liftUp()   { setTargetPosition(targetPosition + modifierValue()); }
    @Override
    public void liftDown() { setTargetPosition(targetPosition - modifierValue()); }

    @Override
    public void update() {
        double current = motorEx.getCurrentPosition();

        double ff = targetPosition > BOTTOM_ZONE ? coefficients.f : 0;
        double power = controller.calculate(targetPosition, current, ff);

        // rest at the bottom instead of stalling against the stop
        if (targetPosition <= BOTTOM_ZONE && current <= BOTTOM_ZONE + 5) power = 0;

        motorEx.setPower(power);

        telemetry.addData("Lift current", current);
        telemetry.addData("Lift target", targetPosition);
        telemetry.addData("Lift power", power);
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
        controller.reset();
    }

    // base class keeps `modifier` private; expose it with a protected getter there
    private int modifierValue() { return 40; }
}