package org.firstinspires.ftc.teamcode.Decode.Robot_Competition_20525.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Decode.Robot_Competition_20525.Util.Robot_Composition;


@Configurable
public class Sphere_Operator {
//    public static long intake_switch_cool = 500;
    public enum Ball_Handling_Side {
        Left,
        Right
    }

    public static double left_distance = 15;
    public static double right_distance = 15;
//    public static ArrayList<Float> right_distance_seen = new ArrayList<>();
//    public static ArrayList<Float> left_distance_seen = new ArrayList<>();

    public static double LEFT_POSITION = 0.5962; //the left and right position for the diverter servo
    public static double RIGHT_POSITION = 0;
    public static boolean forcing_divert = false;
    public static long force_divert_time = 3000;
    public static long last_forced_divert = 0;

    public static void force_divert(Robot_Composition robot, boolean left) {
        forcing_divert = true;
        if (left) {
            robot.diverter.setPosition(LEFT_POSITION);
        } else {
            robot.diverter.setPosition(RIGHT_POSITION);
        }
        last_forced_divert = System.currentTimeMillis();
    }

    public static void handle_divert(Robot_Composition robot,double left_sensed_distance, double right_sensed_distance) {
        robot.telemetry.addData("Left Distance", Math.round(left_sensed_distance));
        robot.telemetry.addData("Right Distance", Math.round(right_sensed_distance));
        if (!(forcing_divert && System.currentTimeMillis() + last_forced_divert > force_divert_time) || System.currentTimeMillis() + last_forced_divert > force_divert_time) {
            forcing_divert = false;
            boolean left_condition = left_sensed_distance <= left_distance;
            boolean right_condition = right_sensed_distance <= right_distance;
            robot.telemetry.addData("Left Ball?", left_condition);
            robot.telemetry.addData("Right Ball?", right_condition);
            //TODO Fix distance (low priority)
            if (!left_condition && !right_condition || left_condition && right_condition) {
                return;
            }

            if (left_condition && !right_condition) {
                robot.diverter.setPosition(RIGHT_POSITION);
                return;
            }
            robot.diverter.setPosition(LEFT_POSITION);
        }
    }
//    public static Sphere sphere_in_maw(Robot_Composition robot) {
//        //TODO Fix distance (low priority)
//        // why is the distance incorrect? could be issues with the color sensors. maybe we should recalibrate? will research
//
//        if (robot.color1.getDistance(DistanceUnit.CM) < left_distance) {
//            robot.telemetry.addData("LEFT DISTANCE", robot.color1.getDistance(DistanceUnit.CM));
//            robot.telemetry.addData("LEFT CONDITION", robot.color1.getDistance(DistanceUnit.CM) < left_distance);
//            robot.telemetry.addLine("LEFT MESSING UP");
//            return new Sphere(
//                    new Sphere_Handling_Constants(
//                            Ball_Handling_Side.Left,
//                            System.currentTimeMillis()
//            ));
//        }
//        if (right_distance < right_distance) {
//            robot.telemetry.addData("RIGHT DISTANCE", right_distance);
//            robot.telemetry.addData("RIGHT CONDITION", right_distance < right_distance);
//            robot.telemetry.addLine("RIGHT MESSING UP");
//            return new Sphere(
//                    new Sphere_Handling_Constants(
//                            Ball_Handling_Side.Right,
//                            System.currentTimeMillis()
//                    ));
//        }
//        return null;
//    }

    public static class Sphere_Handling_Constants {
        public Ball_Handling_Side side;
        public long time_stamp;

        public Sphere_Handling_Constants(Ball_Handling_Side s, long t) {
            side = s;
            time_stamp = t;


        }

    }

    public static class Sphere {
        public Sphere_Handling_Constants constants;
        public Sphere (Sphere_Handling_Constants c) {
            constants = c;
        }
    }
//    public static class Sphere_Color {
//        public double alpha;
//        public double red;
//        public double green;
//        public double blue;

//        public static Sphere_Color Green = new Sphere_Color(1,1,1,0);
//        public static Sphere_Color Purple = new Sphere_Color(1,0,1,1);
//        public Sphere_Color (double a, double r, double g, double b) {
//            alpha = a;
//            red = r;
//            green = g;
//            blue = b;
//        }
//        public Sphere_Color (NormalizedRGBA rgba) {
//            alpha = rgba.alpha;
//            red = rgba.red;
//            green = rgba.green;
//            blue = rgba.blue;
//        }
//        public Sphere_Color (double argb) {
//            String rgbstring = Double.toString(argb);
//            double length = rgbstring.length() / 3.0;
//            alpha = Double.parseDouble(rgbstring.substring(0, (int) length));
//            red = Double.parseDouble(rgbstring.substring((int) length, (int) (length * 2)));
//            green = Double.parseDouble(rgbstring.substring((int) (length * 2), (int) (length * 3)));
//        }
//    }

}
