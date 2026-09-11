package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.ToggledMotor;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems.shootSpeedController;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleUnaryOperator;

public class Teleop extends OpMode {
    MecanumDrive mecanumDrive;
    shootSpeedController shootSpeedController;
    Flywheel flywheel;
    ToggledMotor intake;
    NeuronLocalizer localizer;
    NeuronDrive drive;
    DoubleUnaryOperator regression = d -> d + 3;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0,0,0,0);
    @Override
    public void init() {
        localizer = new NeuronLocalizer(hardwareMap,telemetry,0,0);
        drive = new NeuronDrive(hardwareMap,localizer,14);
        localizer.setPosition(new Pose2D(0,0,0));
        mecanumDrive = new MecanumDrive(hardwareMap,telemetry,gamepad1);
        flywheel = new Flywheel(hardwareMap,telemetry,pidfCoefficients);
        shootSpeedController = new shootSpeedController(flywheel,regression,localizer,new Pose2D(0,0,0));
        intake = new ToggledMotor(hardwareMap,telemetry,"intake",0.5, TimeUnit.SECONDS);
    }

    @Override
    public void loop() {
//        drive.update();
        controls();
        shootSpeedController.update();
        intake.update();
        mecanumDrive.update();
        flywheel.update();
    }

    public void controls() {
        if (gamepad1.aWasPressed()) {
            intake.toggle();
        }
    }

}
