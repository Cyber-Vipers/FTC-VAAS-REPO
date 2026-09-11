package org.firstinspires.ftc.teamcode.Decode.Runt.opmodes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "Initialize Limelight for Config")

public class init_limelight extends OpMode {

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    public Limelight3A limelight;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {

    }
}
