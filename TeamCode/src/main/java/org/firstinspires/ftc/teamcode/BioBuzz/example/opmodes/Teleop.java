package org.firstinspires.ftc.teamcode.BioBuzz.example.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Jiamin Tang")
public class Teleop extends OpMode {

    public DcMotorEx flywheel,intake;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class,"frontLeft");
        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            flywheel.setVelocity(1500);
            intake.setPower(1);
        }
        if (gamepad1.x) {
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (gamepad1.b) {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }




}
