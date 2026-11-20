package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.LiftConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.TeleOpConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.FieldConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.LiftShooter;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

import java.util.Locale;

/**
 * Builds the shot table: distance to the goal -> (lift position, flywheel speed).
 *
 *   1. Put the robot on the reset spot and press Back to set odometry there.
 *   2. Drive somewhere and hold RB to face the goal.
 *   3. Adjust the lift (dpad up/down) and flywheel (dpad right/left) and fire with A until
 *      shots go in reliably. Left stick button jumps to what the current fit predicts here.
 *   4. Press X to record (distance, lift, speed). Do 4-6+ distances from nearest to farthest.
 *   5. Press Start to fit and save. Every OpMode loads the saved fit at Init. Then copy the
 *      printed lines into Constants.ShotTableConstants so a fresh Control Hub has them too.
 *
 * gamepad1:
 *   sticks / LB      drive / slow
 *   RB (hold)        auto-aim at the goal
 *   dpad up/down     lift +/- LIFT_STEP ticks (hold Y for LIFT_FINE_STEP)
 *   dpad right/left  flywheel +/- SPEED_STEP ticks/s (hold Y for SPEED_FINE_STEP)
 *   A                fire right away, ready or not (the status shows where lift + flywheel were)
 *   X / B            record a point / delete the last one
 *   Start            fit + save
 *   left stick btn   jump to the fit's prediction for this distance
 *   right stick btn  HAND MODE: lift and drivetrain go limp so you can push the lift to a
 *                    position and the robot to a spot by hand; both keep tracking. Press again
 *                    and the lift holds where you left it. Firing and recording work in hand mode.
 *   LT (hold)        intake: flywheel spins in, transfer + intake pull in, lift stays put
 *   RT (hold)        stop the flywheel and transfer
 *   Back             reset odometry to the reset spot
 * During Init: X = blue, B = red. Saved points are loaded, so a session can add to the last one.
 */
@TeleOp(name = "HoneyComb Shot Tuner", group = "Honeycomb Tuning")
public class HoneyCombShotTuner extends LinearOpMode {
    public static int LIFT_STEP = 25, LIFT_FINE_STEP = 5, SPEED_STEP = 50, SPEED_FINE_STEP = 10;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1);
        NeuronLocalizer localizer = LiftShooter.createLocalizer(hardwareMap, telemetry);
        localizer.calibrate(true); // odometry starts at 0,0: press Back on the reset spot
        LiftShooter robot = new LiftShooter(hardwareMap, telemetry, drive, localizer);
        ShotTable table = robot.shotTable;

        double liftTicks = ShooterConstants.FIXED_LIFT_TICKS;
        double speedTps = ShooterConstants.FIXED_FLYWHEEL_TPS;
        boolean firing = false;
        double firedMs = 0;
        ElapsedTime feedTimer = new ElapsedTime();
        String status = table.points.size() + " saved points loaded";

        while (opModeInInit()) {
            if (gamepad1.xWasPressed()) robot.alliance = FieldConstants.Alliance.BLUE;
            if (gamepad1.bWasPressed()) robot.alliance = FieldConstants.Alliance.RED;
            telemetry.addLine("X = blue, B = red");
            telemetry.addData("alliance", robot.alliance);
            telemetry.addData("status", status);
            telemetry.update();
        }

        while (opModeIsActive()) {
            Pose2D pose = robot.beginLoop(); // bulk read + the one Pinpoint read

            // Read every press each loop: a *WasPressed() left unread fires later, when it's read.
            boolean firePressed = gamepad1.aWasPressed();
            boolean predictPressed = gamepad1.leftStickButtonWasPressed();
            boolean fine = gamepad1.y;
            if (gamepad1.dpadUpWasPressed()) liftTicks += fine ? LIFT_FINE_STEP : LIFT_STEP;
            if (gamepad1.dpadDownWasPressed()) liftTicks -= fine ? LIFT_FINE_STEP : LIFT_STEP;
            if (gamepad1.dpadRightWasPressed()) speedTps += fine ? SPEED_FINE_STEP : SPEED_STEP;
            if (gamepad1.dpadLeftWasPressed()) speedTps -= fine ? SPEED_FINE_STEP : SPEED_STEP;
            liftTicks = Range.clip(liftTicks, LiftConstants.LOWEST_TICKS, LiftConstants.HIGHEST_TICKS);
            speedTps = Range.clip(speedTps, 0, FlywheelConstants.MAX_TPS);

            if (gamepad1.backWasPressed()) {
                robot.resetOdometry();
                pose = FieldConstants.resetPose(robot.alliance);
                status = "odometry reset to the " + robot.alliance + " reset spot";
            }
            if (gamepad1.rightStickButtonWasPressed()) {
                boolean limp = !robot.lift.isLimp();
                robot.lift.setLimp(limp);
                drive.setCoast(limp);
                status = limp ? "HAND MODE: push the lift and the robot by hand"
                        : String.format(Locale.US, "lift holding at %.0f, driving back on", robot.lift.getPosition());
            }
            boolean handMode = robot.lift.isLimp();
            double distance = FieldConstants.distanceToGoal(pose, robot.alliance);

            // ---- aim -------------------------------------------------------------------
            boolean aimed = false;
            if (!handMode && gamepad1.right_bumper) {
                aimed = robot.aim.update(pose, FieldConstants.headingToGoal(pose, robot.alliance));
            } else {
                robot.aim.stop();
            }

            // ---- lift ------------------------------------------------------------------
            if (handMode) {
                // The lift goes wherever it's pushed; record and fire use where it really is.
                liftTicks = Range.clip(robot.lift.getPosition(), LiftConstants.LOWEST_TICKS, LiftConstants.HIGHEST_TICKS);
            } else {
                if (predictPressed) {
                    liftTicks = table.liftTicks(distance);
                    speedTps = table.flywheelTps(distance);
                    status = "jumped to the fit's prediction for this distance";
                }
                robot.lift.setTargetPosition((int) Math.round(liftTicks));
            }

            // ---- flywheel, transfer, intake ----------------------------------------------
            double feedDt = feedTimer.milliseconds();
            feedTimer.reset();
            if (gamepad1.left_trigger >= TeleOpConstants.TRIGGER_THRESHOLD) {
                firing = false;
                robot.flywheel.setTarget(-FlywheelConstants.INTAKE_TPS);
                robot.intake.setReverse();
                robot.intake.On();
                robot.liftTransfer.setReverse();
                robot.liftTransfer.On();
            } else if (gamepad1.right_trigger >= TeleOpConstants.TRIGGER_THRESHOLD) {
                firing = false;
                robot.flywheel.setTarget(0);
                robot.intake.Off();
                robot.liftTransfer.Off();
            } else {
                robot.flywheel.setTarget((int) Math.round(speedTps));
                if (firePressed && !firing) {
                    // Fires right away, on target or not; the status shows where the lift and the
                    // flywheel actually were, so a miss can be blamed on them not being there yet.
                    firing = true;
                    firedMs = 0;
                    status = String.format(Locale.US, "fired at lift %.0f (set %.0f), flywheel %.0f (set %.0f)%s",
                            robot.lift.getPosition(), liftTicks, robot.flywheel.getVelocity(), speedTps,
                            robot.lift.atTarget() && robot.flywheel.isUp2Speed() ? "" : "  -- NOT on target yet");
                }
                if (firing && firedMs > ShooterConstants.FIRE_TIME_MS) firing = false;
                boolean recovering = ShooterConstants.FIRE_RECOVER_TPS > 0
                        && robot.flywheel.getVelocity() < robot.flywheel.getTargetVelocity() - ShooterConstants.FIRE_RECOVER_TPS;
                if (!firing) {
                    robot.intake.Off();
                    robot.liftTransfer.Off();
                } else {
                    robot.intake.setReverse();
                    robot.intake.On();
                    robot.liftTransfer.setForward();
                    if (recovering) {
                        robot.liftTransfer.Off();
                    } else {
                        robot.liftTransfer.On();
                        firedMs += feedDt;
                    }
                }
            }
            robot.updateHardware();
            drive.update();

            // ---- data ------------------------------------------------------------------
            if (gamepad1.xWasPressed()) {
                table.points.add(new ShotTable.DataPoint(distance, liftTicks, speedTps));
                status = String.format(Locale.US, "recorded #%d: %.1f in, lift %.0f, flywheel %.0f",
                        table.points.size(), distance, liftTicks, speedTps);
            }
            if (gamepad1.bWasPressed() && !table.points.isEmpty()) {
                table.points.remove(table.points.size() - 1);
                status = "removed the last point, " + table.points.size() + " left";
            }
            if (gamepad1.startWasPressed()) {
                if (table.fit()) {
                    table.save();
                    status = "fit + saved to the Control Hub (" + ShotTable.FILE_NAME + ")";
                    telemetry.log().add("Paste into Constants.ShotTableConstants:");
                    for (String line : table.coefficientsAsJava().split("\n")) telemetry.log().add(line);
                } else {
                    status = "fit failed: record at least one point";
                }
            }

            // ---- telemetry -------------------------------------------------------------
            telemetry.addData("status", status);
            telemetry.addData("loop", "%.0f ms", robot.loopMs);
            telemetry.addData("alliance", robot.alliance);
            telemetry.addData("pose (in)", "x %.1f  y %.1f  h %.1f°%s", pose.x, pose.y, Math.toDegrees(pose.h),
                    handMode ? "  (pushing by hand)" : "");
            telemetry.addData("distance to goal (in)", "%.1f  (%s CELL)", distance,
                    FieldConstants.targetCell(pose, robot.alliance));
            telemetry.addData("aim", gamepad1.right_bumper && !handMode
                    ? String.format(Locale.US, "%s (%.1f°)", aimed ? "aimed" : "turning", robot.aim.errorDeg) : "hold RB");
            telemetry.addData("set", "lift %.0f, flywheel %.0f", liftTicks, speedTps);
            robot.lift.addTelemetry(telemetry);
            robot.flywheel.addTelemetry(telemetry);
            telemetry.addData("fit predicts here", "lift %.0f, flywheel %.0f", table.liftTicks(distance), table.flywheelTps(distance));
            telemetry.addData("points", table.points.size());
            if (!Double.isNaN(table.liftR2)) {
                telemetry.addData("R² lift / flywheel", "%.3f / %.3f", table.liftR2, table.flywheelR2);
            }
            for (int i = Math.max(0, table.points.size() - 5); i < table.points.size(); i++) {
                ShotTable.DataPoint p = table.points.get(i);
                telemetry.addLine(String.format(Locale.US, "  #%d  %.1f in  lift %.0f  flywheel %.0f",
                        i + 1, p.distance, p.liftTicks, p.flywheelTps));
            }
            telemetry.addLine("RB aim | dpad lift/speed (Y fine) | A fire | X record | B undo | Start fit | L3 predict | R3 hand");
            telemetry.update();
        }
    }
}
