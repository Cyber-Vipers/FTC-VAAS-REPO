package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

public class Pinpoint {
    private Telemetry telemetry;
    public GoBildaPinpointDriver pinpoint;


    public Pinpoint(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

    }

    public Pose2D getPose() {
        pinpoint.update();
        return new Pose2D(pinpoint.getPosition().getX(DistanceUnit.INCH), pinpoint.getPosition().getY(DistanceUnit.INCH), pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
    }

//    public Pose2D getVel() {
//        return new Pose2D(pinpoint.getVelocity().x, pinpoint.getVelocity().y, pinpoint.getVelocity().h);
//    }

    public void setPosition(org.firstinspires.ftc.robotcore.external.navigation.Pose2D pos) {
        pinpoint.setPosition(pos);
        pinpoint.update();
    }
    public void setPosition(Pose2D pos) {
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose =
                new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.INCH, pos.x, pos.y, AngleUnit.RADIANS, pos.h);
        setPosition(pose);
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(-96, -96, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();

        pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0
        ));

    }


    public void setFarRed() {
        setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.INCH, 24, -62.5, AngleUnit.DEGREES, 90));
    }
    public void setFarBlue() {
        setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.INCH, -24, -62.5, AngleUnit.DEGREES, 90));

    }
    public void setCloseRed() {

    }
    public void setCloseBlue() {

    }
}
