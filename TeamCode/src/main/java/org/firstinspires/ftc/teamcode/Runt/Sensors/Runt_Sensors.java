package org.firstinspires.ftc.teamcode.Runt.Sensors;

import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Runt_Sensors {

        public enum april_type {
            obelisk,
            red_goal,
            blue_goal
        }
        public Integer[] AprilTag_IDs = {20,21,22,23,24};


    public Limelight3A Limelight;

    public Runt_Sensors(Limelight3A limelight){
        this.Limelight = limelight;
    }


    public void detect_pieces() {
        

    }

}
