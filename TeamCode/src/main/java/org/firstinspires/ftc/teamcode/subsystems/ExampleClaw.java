package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A minimal example subsystem: one servo, two positions. This is a template for your
 * own game-specific mechanisms (intake, arm, shooter, whatever) -- see
 * docs/subsystems.md for how to write, wire up, and call subsystems like this one
 * from both TeleOp and Autonomous, alongside NeuronDrive.
 *
 * The pattern to copy:
 *  - constructor takes HardwareMap (and Telemetry if you need to report state) and owns
 *    all of this mechanism's hardware -- nothing outside this class should ever call
 *    hardwareMap.get(...) for this servo/motor.
 *  - public methods (open()/close()/toggle() here, whatever verbs make sense for your
 *    mechanism) are what your OpModes call directly.
 *  - update() is for state that needs to run every loop even when nothing just changed
 *    (a PID, a timed sequence, telemetry). This example doesn't need one, but it's here
 *    to show where it goes -- see the Turret/Shooter-style subsystems in your season
 *    repo for a real example of update() doing real work.
 */
public class ExampleClaw {
    public enum State { OPEN, CLOSED }

    private final Servo servo;
    public State state = State.CLOSED;

    private static final double OPEN_POS = 0.55;
    private static final double CLOSED_POS = 0.1;

    public ExampleClaw(HardwareMap hwMap, String servoName) {
        servo = hwMap.get(Servo.class, servoName);
        close();
    }

    public ExampleClaw(HardwareMap hwMap) {
        this(hwMap, "claw");
    }

    public void open() {
        servo.setPosition(OPEN_POS);
        state = State.OPEN;
    }

    public void close() {
        servo.setPosition(CLOSED_POS);
        state = State.CLOSED;
    }

    public void toggle() {
        if (state == State.OPEN) close(); else open();
    }

    /** Nothing to do every loop for a simple servo, but real subsystems often need this. */
    public void update() {
        // e.g. run a PID, advance a timed sequence, push telemetry -- see docs/subsystems.md
    }
}
