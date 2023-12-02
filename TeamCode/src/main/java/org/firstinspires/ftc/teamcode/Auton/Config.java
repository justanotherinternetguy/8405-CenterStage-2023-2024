package org.firstinspires.ftc.teamcode.Auton;

@com.acmerobotics.dashboard.config.Config
public class Config {
    public static double liftP = 0.05;
    public static double liftI = 0.03;
    public static double liftD = 0;
    public static double BACKSPOT = 35.5;
    public static int liftTolerance = 10;
    public static double liftMotorFloor = 0.3;
    public static double gravity = 0.5;
    public static boolean fieldCentric = false;
    public static double XMULTI = 1.5;
    public static boolean manualWhite = false;
    public static int temp = 3334;
    public static double translationP = 0.3;
    public static double translationI = 0.02;
    public static double translationD = 0.04;
    public static double rotationP = 0.07;
    public static double rotationI = 0.0;
    public static double rotationD = 0.005;
    public static double tolerance = 1;
    public static double toleranceH = 3;
    public static double powerMultiplier = 0.76;

    public static double BOARDSPEED = 0.425;

    public static double alignment = 0.2;

    public static double liftMotorPowerMultTeleOp = 0.75;
    public static double liftMotorPowerMacro = 0.5;
    public static double liftMotorPowerHold = 0.275;
    public static double liftMotorPowerAuton = 0.38;
    public static double liftMotorPowerDown = 0.42;
    public static int LIFT_MAX = Integer.MAX_VALUE;
    public static int LIFT_BACK = 1350;
    public static int FLOOR = 39;
    public static double KILLTIME = 500;


    public static int mask1LH = 0;
    public static int mask1LS = 100;
    public static int mask1LV = 80;
    public static int mask1UH = 10;
    public static int mask1US = 255;
    public static int mask1UV = 255;

    public static int mask2LH = 100;
    public static int mask2LS = 100;
    public static int mask2LV = 100;
    public static int mask2UH = 255;
    public static int mask2US = 255;
    public static int mask2UV = 255;

    public static double topServoClose = 0.15;
    public static double topServoOpen = 1;
    public static double bottomServoClose = 0.2;
    public static double bottomServoOpen = 1;
    public static double clawServoFloor = 0.775;
    public static double clawServoBackboard = 1;


    public static int burstDelay = 500;

    public static double frontMulti = 1;
}
