package org.firstinspires.ftc.teamcode;

public class utils {
    public static double angleDifference(double from, double to)
    {
        double diff = (from - to + 180 ) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }
    public static double inToM(double inches)
    {
        return inches/39.37;
    }
}
