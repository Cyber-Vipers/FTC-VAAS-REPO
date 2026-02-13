package org.firstinspires.ftc.teamcode.Runt.teaching;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Testing Motor")
@Disabled
public class test_0 extends OpMode {
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    DcMotorEx motor;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "backLeft");

    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        motor.setVelocity(150);
        
        telemetry.addData("Current", motor.getCurrent(CurrentUnit.MILLIAMPS));
    }
}
