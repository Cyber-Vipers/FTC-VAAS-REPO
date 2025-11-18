package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

public class OTOS {
    private Telemetry telemetry;
    private SparkFunOTOS otos;


    public OTOS(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        otos = hwMap.get(SparkFunOTOS.class, "otos");
        configureOtos();
    }

    public Pose2D getPose() {

        return new Pose2D(otos.getPosition().x, otos.getPosition().y, otos.getPosition().h);
    }

    public Pose2D getVel() {
        return new Pose2D(otos.getVelocity().x, otos.getVelocity().y, otos.getVelocity().h);
    }

    public void setPosition(SparkFunOTOS.Pose2D pos) {
        otos.setPosition(pos);
    }
    public void setPosition(Pose2D pos) {
        SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D(pos.x, pos.y, pos.h);
        setPosition(pose);
    }

    public void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-0.2, -6.5, Math.PI * 1.5);
        otos.setOffset(offset);
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);
    }


    public void setFarRed() {
        setPosition(new Pose2D(24, -62.5, 90));
    }
    public void setFarBlue() {
        setPosition(new Pose2D(-24, -62.5, 90));

    }
    public void setCloseRed() {

    }
    public void setCloseBlue() {

    }
}
