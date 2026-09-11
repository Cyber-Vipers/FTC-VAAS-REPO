package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

public class BangBangController {

    //current heading , target heading, value

    public static double BangBangChicken(double cur, double vel,double f,double max_power){
            // we need a way to ensure values above vel are corrected
            // as of right now if vel == 0, then unless cur <= -0 then it won't correct
            if (cur < vel) {
                return max_power;
            } else {
                return f*vel;
            }
    }
    public static double BangBangChicken(double cur, double vel,double f,boolean angle){
        // we need a way to ensure values above vel are corrected
        // as of right now if vel == 0, then unless cur <= -0 then it won't correct

        if (Math.toDegrees(cur) + 180 < Math.toDegrees(vel) + 180) {
            return 1;
        } else {
            return f * vel;
        }
    }

}
