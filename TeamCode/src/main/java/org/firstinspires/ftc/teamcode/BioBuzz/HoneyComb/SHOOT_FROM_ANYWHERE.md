# HoneyComb: shoot from anywhere

One press of RT now turns the robot to the goal, sets the lift and flywheel for the distance, and fires
when all three are ready. Every other control is the same as before. All numbers live in
`Constants.java` (and field positions in `FieldConstants.java`).

## Controls (Honeycomb Robot Teleop, gamepad 1)

| Control | What it does |
| --- | --- |
| Left stick / right stick X / LB | Drive / turn / slow mode, as before. The left stick keeps working while a shot lines up |
| LT (hold) | Intake, as before. Also cancels a shot that is lining up |
| Y (hold) | Outtake, as before. Also cancels a shot that is lining up |
| RT (press) | One-button shot: spin up, aim + lift, fire, back to normal |
| RT again | During a shot that's lining up: fire as soon as the lift is up, without waiting for aim or flywheel |
| Right stick | Moving it cancels a shot that is lining up (the robot is turning itself then) |
| Back | Reset odometry. The robot must be on the alliance's reset spot |
| X / B during Init | Blue / red alliance |

Backup: set `ShooterConstants.USE_ODOMETRY = false` and RT goes back to the old fixed shot
(lift 935, flywheel 2200, no turning).

## Setup, in this order

The robot shoots the old fixed shot (935 / 2200) while turning to the goal until step 6 is done.

1. **Config:** add a "goBILDA Pinpoint Odometry Computer" named `pinpoint` on an I2C port. (The
   copied `drivers/GoBildaPinpointDriver.java` was removed: the SDK already has this driver.)
   **Check the drive first:** this uses MecanumDrive's own motor directions (right side reversed),
   as every TeleOp did up to 11-18. `fb0c1f6` also built a NeuronDrive in Teleop, which reverses the
   left side too. If the robot only drives right that way, set `DriveConstants.REVERSE_LEFT_MOTORS = true`.
2. **HoneyComb Localizer Test:** push forward -> x up, left -> y up, counter-clockwise -> heading up.
   Push 48 in along a tape (should read ~48), spin 5 turns in place (x/y should stay near 0). Fix
   `OdometryConstants` until both pass.
3. **Reset spot:** set `FieldConstants.ROBOT_HALF_LENGTH_IN` / `ROBOT_HALF_WIDTH_IN` to your robot.
   The reset spot is the alliance's loading-zone corner, back on its own alliance wall.
4. **HoneyComb Goal Tuner:** sight the goal from 3-5 spots and paste the printed line into `FieldConstants`.
   Hold RB to check the aim. If the robot turns the wrong way, flip `DriveConstants.AIM_TURN_REVERSED`.
   Tune `AIM_KP` first, then `AIM_KD` to stop overshoot.
5. **HoneyComb Lift Hold Finder** (if the lift pivots): measures the hold power across the lift's range.
   Paste the printed lines into `LiftConstants`. If the lift overshoots when moving between positions,
   try `MAX_UP_SPEED_TPS = 1500`.
6. **HoneyComb Shot Tuner:** at 4-6+ distances, adjust lift and flywheel until shots go in, record each,
   then fit + save (Start). Paste the printed lines into `ShotTableConstants`.
7. **Match TeleOp:** the telemetry's `shot` line says what a shot is waiting on (`aim`, `flywheel`,
   `lift`), and `loop` shows the loop time.

## What changed and why

- **Bugs fixed:** the flywheel was sent 2-3 different speeds every loop (including 0 while shooting);
  `Lift.notablePositions` was `static`, so it grew every Init and kept the first OpMode's value.
- **Loop time:** one bulk read per hub per loop (`MANUAL` caching cleared in `LiftShooter.beginLoop()`),
  one Pinpoint read per loop, the flywheel velocity only sent when it changes, and the transfer's
  current alert set once instead of every loop.
- **Lift:** hold table (any position, not just one), optional smooth moves up, optional hold deadband,
  and it re-zeroes once it has rested at the bottom, so it doesn't have to start all the way down.
