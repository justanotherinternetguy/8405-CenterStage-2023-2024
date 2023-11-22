package org.firstinspires.ftc.teamcode;

public class utils {
    public static double angleDifference(double from, double to) {
        double diff = (from - to + 180) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }

    public static double inToM(double inches) {
        return inches / 39.37;
    }

    // normalize radians to be between -pi and pi
    // for Odometry.java
    public static double normalizeRadians(double radians) {
        double temp = (radians + Math.PI) / (2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5) * 2.0 * Math.PI;
    }
}
