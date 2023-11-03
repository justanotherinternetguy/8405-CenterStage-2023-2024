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

    public static double alignment = 0.2;

    public static double liftMotorPowerMultTeleOp = 0.75;
    public static double liftMotorPowerMacro = 0.8;
    public static double LIFT_MAX = 2900;
}
