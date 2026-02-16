package org.firstinspires.ftc.teamcode.util;

public class BangBangController {


    //current heading , target heading, value
    public static double BangBangChicken(double cur, double vel,double f){
            // we need a way to ensure values above vel are corrected
            // as of right now if vel == 0, then unless cur <= -0 then it won't correct

            if (Math.abs(cur) > vel) {
                return 1;
            } else {
                return f * vel;
            }
    }


}
