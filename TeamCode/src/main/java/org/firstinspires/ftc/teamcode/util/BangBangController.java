package org.firstinspires.ftc.teamcode.util;

public class BangBangController {


    //current heading , target heading, value
    public static double BangBangChicken(double cur, double vel,double f){
            // we need a way to ensure values above vel are corrected
            // as of right now if vel == 0, then unless cur <= -0 then it won't correct

            if (cur > vel) {
                return 1;
            } else {
                return f * vel;
            }
    }
    public static double BangBangChicken(double cur, double vel,double f,boolean angle){
        // we need a way to ensure values above vel are corrected
        // as of right now if vel == 0, then unless cur <= -0 then it won't correct

        if (Math.toDegrees(cur) + 180 > Math.toDegrees(vel) + 180) {
            return 1;
        } else {
            return f * vel;
        }
    }

}
