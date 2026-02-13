package org.firstinspires.ftc.teamcode.Runt.opmodes;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;


import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.RuntOdometryWheelsConstants;

@TeleOp(name="Runt Line")
@Disabled
public class Runt_Line extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {}

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        follower = RuntOdometryWheelsConstants.createFollower(hardwareMap);
        follower.update();
    }

    @Override
    public void start() {
        follower = RuntOdometryWheelsConstants.createFollower(hardwareMap);
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(0,0), new Pose(DISTANCE,0)));
        forwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();


        if (!follower.isBusy()) {
            stop();
        } else {

        }

    }
}