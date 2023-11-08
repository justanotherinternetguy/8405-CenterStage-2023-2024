package org.firstinspires.ftc.teamcode.Auton;

@com.acmerobotics.dashboard.config.Config
public class Config {
    public static double translationP = 0.08;
    public static double translationI = 0.0;
    public static double translationD = 0.006;
    public static double rotationP = 0.03;
    public static double rotationI = 0.0;
    public static double rotationD = 0.002;
    public static double tolerance = 1.5;
    public static double powerMultiplier = 0.8;

    public static double alignment = 0.2;

    public static double liftMotorPowerMultTeleOp = 0.75;
    public static double liftMotorPowerMacro = 0.4;
    public static double liftMotorPowerHold = 0.3;
    public static double LIFT_MAX = 1500;
    public static double LIFT_MIN = 25;
    public static int FLOOR = 25;
    public static double KILLTIME = 500;


    public static int mask1LH = 0;
    public static int mask1LS = 100;
    public static int mask1LV = 100;
    public static int mask1UH = 10;
    public static int mask1US = 255;
    public static int mask1UV = 255;

    public static int mask2LH = 100;
    public static int mask2LS = 100;
    public static int mask2LV = 100;
    public static int mask2UH = 255;
    public static int mask2US = 255;
    public static int mask2UV = 255;

    public static double topServoClose = -0.3;
    public static double topServoOpen = -0.5;

    public static double bottomServoClose = 0.875;
    public static double bottomServoOpen = 0.75;

    public static int burstDelay = 500;

    public static double frontMulti = 0.9;
}
