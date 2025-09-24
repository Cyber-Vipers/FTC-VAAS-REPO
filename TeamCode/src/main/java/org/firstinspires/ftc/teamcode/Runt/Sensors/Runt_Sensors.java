package org.firstinspires.ftc.teamcode.Runt.Sensors;

import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Runt_Sensors {
    static class april_check {
        public enum april_type {
            obelisk,
            red_goal,
            blue_goal
        }

    }
    public Limelight3A Limelight;

    public Runt_Sensors(Limelight3A limelight){
        this.Limelight = limelight;
    }


    public april_check  check_apriltags(){
        return new april_check();
    }

}
