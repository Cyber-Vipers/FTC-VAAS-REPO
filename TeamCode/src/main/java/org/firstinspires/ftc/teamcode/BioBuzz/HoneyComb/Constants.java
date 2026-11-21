package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Every HoneyComb number someone might want to adjust, in one place. Change it here (or live in
 * Panels), rebuild, and every OpMode uses the new value. Each section belongs to one class.
 * Inside a section the values you'll change most come first, and measured hardware setup
 * (config names, directions, offsets) comes last.
 *
 * Fields are static but NOT final on purpose, so Panels and the tuner OpModes can change them
 * live. Live changes are lost when the Robot Controller app restarts, so copy good values here.
 *
 * Not in here: field positions (FieldConstants) and the fitted shot table, which the
 * HoneyComb Shot Tuner saves to the Control Hub (ShotTable).
 */
public final class Constants {
    private Constants() {}

    // ============================================================================================
    /** subsystems/LiftShooter.java: the one-button shot. */
    @Configurable
    public static class ShooterConstants {
        /**
         * true: RT aims at the goal and takes lift position + flywheel speed from the shot table.
         * false: RT does the old fixed shot (FIXED_LIFT_TICKS, FIXED_FLYWHEEL_TPS) with no turning.
         */
        public static boolean USE_ODOMETRY = true;
        /** The old fixed shot (Teleop used 935 / 2200). Also the shot table's default until it's tuned. */
        public static int FIXED_LIFT_TICKS = 935;
        public static int FIXED_FLYWHEEL_TPS = 2200;

        /** On RT the flywheel spins up first; the lift raises and the robot turns once it's at speed or this long. */
        public static double SPINUP_LEAD_MS = 500;
        /**
         * How long the transfer feeds for one shot. Starting guess: measure how long it takes to
         * empty the lift and add a little.
         */
        public static double FIRE_TIME_MS = 1500;
        /** A shot always ends this long after it starts firing, even if FIRE_RECOVER_TPS kept pausing it. */
        public static double FIRE_MAX_MS = 3000;
        /** Give up on a shot that hasn't fired in this long (back to NORMAL). */
        public static double SHOT_TIMEOUT_MS = 5000;

        /** Fire once the heading is within this of the goal... */
        public static double AIM_TOLERANCE_DEG = 2;
        /** ...or within this once it has been aiming for AIM_RELAX_MS, so a shot never stalls on the last degree. */
        public static double AIM_RELAXED_TOLERANCE_DEG = 5, AIM_RELAX_MS = 1000;
        /** Only fire while the robot moves slower than this (in/s). 0 = off. */
        public static double FIRE_MAX_SPEED_IN_S = 0;
        /** While firing, pause the transfer whenever the flywheel is more than this below target (TPS). 0 = off. */
        public static double FIRE_RECOVER_TPS = 0;
    }

    // ============================================================================================
    /** subsystems/ShotTable.java: used until the Shot Tuner has saved a fit to the Control Hub. */
    @Configurable
    public static class ShotTableConstants {
        /** Highest polynomial degree the fit uses (2 = quadratic). */
        public static int MAX_DEGREE = 2;
        /** Paste the Shot Tuner's printed lines here so a fresh Control Hub shoots the same. */
        public static double[] DEFAULT_LIFT_COEFFS = {935};
        public static double[] DEFAULT_FLYWHEEL_COEFFS = {2200};
        public static double DEFAULT_MIN_DIST_IN = 0, DEFAULT_MAX_DIST_IN = 200;
    }

    // ============================================================================================
    /** subsystems/PIDControlledLift.java (and the plain Lift it extends). */
    @Configurable
    public static class LiftConstants {
        /** Lowest and highest positions the lift is allowed to go to. */
        public static int LOWEST_TICKS = 0, HIGHEST_TICKS = 1762;
        /** Within this many ticks of the target counts as "there" for shooting. */
        public static int TOLERANCE_TICKS = 100;

        /** PIDF. F is the hold power when the hold table below is empty. */
        public static PIDFCoefficients PIDF = new PIDFCoefficients(0.0028, 0, 0.0003, 0.039);
        /** Power limits: full power up, gentle down. */
        public static double UP_POWER = 1, DOWN_POWER = 0.3;

        // ---- smooth moves: the PID chases a target that slides toward the real one ----
        /** Down: LANDING_SPEED + sqrt(2 * DOWN_DECEL * distance left), at most MAX_DOWN_SPEED (ticks/s). */
        public static double MAX_DOWN_SPEED_TPS = 1500, LANDING_SPEED_TPS = 150, DOWN_DECEL_TPS2 = 2500;
        /**
         * Up, the same shape. MAX_UP_SPEED_TPS = 0 keeps the old behavior (jump straight to the
         * target at full power). Try 1500 if the lift overshoots moving between shot positions.
         */
        public static double MAX_UP_SPEED_TPS = 0, UP_MIN_SPEED_TPS = 150, UP_DECEL_TPS2 = 2500;
        /** Below this the lift counts as "at the bottom": no hold power, and it rests unpowered. */
        public static int BOTTOM_ZONE_TICKS = 20;

        // ---- holding any position ----
        /**
         * Measured hold power (motor power that keeps the lift still) at several positions, from
         * the HoneyComb Lift Hold Finder. Straight lines in between, flat past the ends. Ticks must
         * go up. Fewer than 2 points = use PIDF.f everywhere (a straight slide only needs that).
         */
        public static double[] HOLD_TABLE_TICKS = {137,425,749,1100,1436};
        public static double[] HOLD_TABLE_POWER = {0.168,0.178,0.132,0.007,-0.121};
        /** Extra power toward the target to get past friction (the Hold Finder measures it). 0 = off. */
        public static double KS = 0;
        /** KS only applies while the lift is further than this from where it should be. */
        public static int KS_MIN_ERROR_TICKS = 10;
        /**
         * Profile finished and within this many ticks: hold with the feedforward only and stop
         * correcting, so the PID doesn't hunt in the slack. Must stay below TOLERANCE_TICKS. 0 = off.
         */
        public static int HOLD_DEADBAND_TICKS = 0;
        /**
         * Re-zero the encoder when the lift has rested at the bottom this long reading below 0, i.e.
         * it wasn't all the way down at Init. (Still: start it down when you can.)
         */
        public static boolean REZERO_AT_BOTTOM = true;
        public static double REZERO_REST_MS = 300;

        /** Positions the Lift Hold Finder measures at. Spread them over the shooting range. */
        public static double[] HOLD_FINDER_TICKS = {200, 500, 800, 1100, 1400};
        /** Hold Finder: power ramp (per second) and how far the lift must move to count as moving. */
        public static double HOLD_FINDER_RAMP = 0.05, HOLD_FINDER_MOVE_TICKS = 15;

        // ---- hardware setup ----
        public static final String NAME = "lift";
        public static boolean REVERSED = false;
    }

    // ============================================================================================
    /** subsystems/Flywheel.java. Speeds are encoder ticks per second (TPS). */
    @Configurable
    public static class FlywheelConstants {
        /** Speed spinning backwards (pulling balls in) whenever it isn't shooting. */
        public static int INTAKE_TPS = 1000;
        /** Within this many TPS of the target counts as "at speed". */
        public static int AT_SPEED_TOLERANCE_TPS = 130;
        /** Velocity PIDF for the hub's built-in controller, from the Flywheel PIDF Auto Tuner. */
        public static PIDFCoefficients PIDF = new PIDFCoefficients(8.509, 0, 0, 11.661); // from d5da670 "fixed PIDF"
        /** Top speed. 32767 / F = 32767 / 12.701 = 2580 TPS; re-measure if the motor or gearing changes. */
        public static int MAX_TPS = 2580;
        public static final String NAME = "flywheel";
    }

    // ============================================================================================
    /** The intake and transfer ToggledMotors, and the liftTransfer servo. */
    @Configurable
    public static class IntakeConstants {
        public static final String INTAKE_NAME = "intake", TRANSFER_NAME = "transfer";
        /** Over-current alert, set once at init; voltageControl() turns the transfer off above it. */
        public static double CURRENT_LIMIT_MA = 5000;
        /** liftTransfer servo positions (toggledServo). */
        public static double INTAKE_LIFT_UP = -0.45, INTAKE_LIFT_DOWN = 0.3;
        public static final String INTAKE_LIFT_NAME = "liftTransfer";
    }

    // ============================================================================================
    /** subsystems/MecanumDrive.java and subsystems/GoalAim.java. */
    @Configurable
    public static class DriveConstants {
        /** Left bumper slow mode multiplier. */
        public static double SLOW_MULTIPLIER = 0.5;
        /** Strafing is a bit weaker than driving forward; this evens it out. */
        public static double STRAFE_FACTOR = 1.1;

        // ---- auto-aim: turn power = AIM_KP * error(deg) - AIM_KD * turn rate(deg/s), + AIM_KS past tolerance ----
        /** Starting values: tune AIM_KP first with the others at 0, then AIM_KD to stop overshoot. */
        public static double AIM_KP = 0.03, AIM_KD = 0.002, AIM_KS = 0;
        public static double AIM_MAX_TURN = 0.7;
        /** Flip if the robot turns AWAY from the goal when aiming (depends on motor directions). */
        public static boolean AIM_TURN_REVERSED = false;

        // ---- hardware setup ----
        /**
         * false = only the right motors reversed, as MecanumDrive always did (and every TeleOp up to
         * 2026-11-18). fb0c1f6 also built a NeuronDrive in Teleop, whose constructor reverses the
         * LEFT motors too; set this true only if the robot drives correctly that way.
         */
        public static boolean REVERSE_LEFT_MOTORS = false;
    }

    // ============================================================================================
    /** goBILDA Pinpoint (NeuronLocalizer), built by LiftShooter.createLocalizer(). */
    @Configurable
    public static class OdometryConstants {
        /**
         * Pod offsets from the robot's center, in mm. X pod (tracks forward): sideways from
         * center, LEFT = positive. Y pod (tracks sideways): forward of center, FORWARD = positive.
         * -101.6 / 0 (= -4 in / 0 in) were hardcoded in NeuronLocalizer, silently overriding these;
         * the old Teleop used 145 / 35.7. Verify with the Localizer Test: spin 5 turns in place and
         * x/y must stay near 0. Wrong offsets make the pose drift every time the robot turns to aim.
         */
        public static double X_POD_OFFSET_MM = -101.6, Y_POD_OFFSET_MM = 0;
        public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        /** Pushing the robot forward must make x go UP, and pushing it left must make y go UP. */
        public static GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public static GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public static final String NAME = "pinpoint";
    }

    // ============================================================================================
    /** opmodes/teleop/Teleop.java: driver controls. */
    @Configurable
    public static class TeleOpConstants {
        /** How far a trigger must be pulled to count as pressed. */
        public static double TRIGGER_THRESHOLD = 0.2;
        /** Moving the right stick further than this cancels a shot that is lining up. */
        public static double STICK_CANCEL_DEADBAND = 0.3;
        /** Show the FIELD DEBUG telemetry block (odometry, reset, CELL choice, aim). */
        public static boolean SHOW_FIELD_DEBUG = true;
    }
}
