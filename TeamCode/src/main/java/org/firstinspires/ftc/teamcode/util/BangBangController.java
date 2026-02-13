package org.firstinspires.ftc.teamcode.util;

public class BangBangController {

    public static double BangBangChicken(double cur, double vel,double f){
        if (cur < vel ) {
            return 1;
        } else {
            return f * vel;
        }
    }


}
