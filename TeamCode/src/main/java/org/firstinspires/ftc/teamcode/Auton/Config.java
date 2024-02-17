package org.firstinspires.ftc.teamcode.Auton;

@com.acmerobotics.dashboard.config.Config
public class Config {
    public static double settlingTime = 250;
    public static double boardDistance = 10;
    public static int boardColumn = 0;
    public static int secondOffset = 125;
    public static double boardPower = 0.35;
    public static double backBoardOffset = 12;
    public static boolean instantEndGame = false; // testing purposes
    public static int boardBase = 425;
    public static double hangPower = -0.85;
    public static double liftP = 0.02;
    public static double liftI = 0;
    public static double liftD = 0;
    public static int liftTolerance = 10;
    public static double gravity = 0.22;
    public static boolean fieldCentric = false;
    public static double XMULTI = 1.5;
    public static boolean manualWhite = false;
    public static int temp = 3334;
    public static double translationP = 0.1;
    public static double translationI = 0.325;
    public static double translationD = 0.03112;
    public static double rotationP = 0.038;
    public static double rotationI = 0.076;
    public static double rotationD = 0.002508;
    public static double tolerance = 0.75;
    public static double toleranceH = 1.5;
    public static double powerMultiplier = 0.8;
    public static double liftMotorPowerMultTeleOp = 1;
    public static double liftMotorPowerMacro = 0.45;
    public static double liftMotorPowerAuton = 0.6;
    public static int liftBackBoard = 550;
    public static int LIFT_MAX = Integer.MAX_VALUE;
    public static int FLOOR = 100;
    public static double KILLTIME = 1000;


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

    public static double bottomServoClose = 0.35;
    public static double bottomServoOpen = 0.43;
    public static double topServoClose = 0.515;
    public static double topServoOpen = 0.59;
    public static double clawServoFloor = 0.8;
    public static double clawServoBackboard = 0.5125;
    public static double frontMulti = 1;
}
