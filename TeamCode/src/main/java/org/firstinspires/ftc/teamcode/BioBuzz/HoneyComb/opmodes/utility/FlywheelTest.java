package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Utility(name = "Flywheel Test")
public class FlywheelTest extends OpMode {
    DcMotorEx m;

    @Override
    public void init() {
        m = hardwareMap.get(DcMotorEx.class, "flywheel");
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.x)      m.setPower(0.2);
        else if (gamepad1.y) m.setPower(-0.2);
        else if (gamepad1.a) m.setVelocity(300);
        else if (gamepad1.b) m.setVelocity(-300);
        else                 m.setPower(0);

        telemetry.addData("Position", m.getCurrentPosition());
        telemetry.addData("Velocity (ticks/s)", m.getVelocity());
        telemetry.addData("Power", m.getPower());
        telemetry.addData("Mode", m.getMode());
        telemetry.addData("Direction", m.getDirection());
    }
}