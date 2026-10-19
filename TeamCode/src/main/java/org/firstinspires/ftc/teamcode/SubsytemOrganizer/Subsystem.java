package org.firstinspires.ftc.teamcode.SubsytemOrganizer;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {

    public void update();


    /**
     *
     * @param value
     * @param tolerance
     * @param comparison
     * @return
     */
    public default boolean plusOrMinus(double value, double tolerance, double comparison) {
        return Math.abs(value - comparison) <= tolerance;
    }


}
