package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RuntOdometryWheelsConstants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.52544)
            .forwardZeroPowerAcceleration(-1987.529256386748)
            .lateralZeroPowerAcceleration(-88.53049322546104)
            .automaticHoldEnd(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.03, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0, 0.1, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.25, 0, 0.03,0, 0))
            .lateralZeroPowerAcceleration(-67.60657954116293);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78.88812063622663)
            .useBrakeModeInTeleOp(true)
            .yVelocity(64.22306955893208);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.25)
            .strafePodX(-0.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
            )

            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            3 // originally 2
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)

                .build();

    }
}
