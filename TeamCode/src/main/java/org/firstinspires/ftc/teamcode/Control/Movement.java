package org.firstinspires.ftc.teamcode.Control;

import android.app.admin.DeviceAdminService;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils;

import java.util.Arrays;
import java.util.function.DoubleBinaryOperator;
import java.util.function.Supplier;

public class Movement {
    private Drive drive;
    private Pose2d pose;
    private PID driveX = new PID(Config.translationP, Config.translationI, Config.translationD);
    private PID driveY = new PID(Config.translationP, Config.translationI, Config.translationD);
    private PID driveH = new PID(Config.rotationP, Config.rotationI, Config.rotationD);
    private double tolX = Config.tolerance;
    private double tolY = Config.tolerance;
    private double tolH = Config.toleranceH;

    public Movement(Drive drive, PID.Config xConfig, PID.Config yConfig, PID.Config hConfig, Double tolX, Double tolY, Double tolH) {
        this.drive = drive;
        if (xConfig != null) this.driveX = new PID(xConfig);
        if (yConfig != null) this.driveY = new PID(yConfig);
        if (hConfig != null) this.driveH = new PID(hConfig);
        if (tolX != null) this.tolX = tolX;
        if (tolY != null) this.tolY = tolY;
        if (tolH != null) this.tolH = tolH;
    }
    public Movement(Drive drive, PID.Config xConfig, PID.Config yConfig, PID.Config hConfig, Double tolT, Double tolH) {
        this(drive, xConfig, yConfig, hConfig, tolT, tolT, tolH);
    }
    public Movement(Drive drive, PID.Config xConfig, PID.Config yConfig, PID.Config hConfig) {
        this(drive, xConfig, yConfig, hConfig, null, null, null);
    }
    public Movement(Drive drive, PID.Config tConfig, PID.Config hConfig) {
        this(drive, tConfig, tConfig, hConfig, null, null, null);
    }
    public Movement(Drive drive, PID.Config tConfig, PID.Config hConfig, Double tolT, Double tolH) {
        this(drive, tConfig, tConfig, hConfig, tolT, tolT, tolH);
    }
    public Movement(Drive drive, Double tolX, Double tolY, Double tolH) {
        this(drive, null, null, null, tolX, tolY, tolH);
    }
    public Movement(Drive drive, Double tolT, Double tolH) {
        this(drive, null, null, null, tolT, tolT, tolH);
    }
    public Movement(Drive drive) {
        this(drive, null, null, null, null, null, null);
    }

    // returns whether or not the bot still needs to move

    public boolean move(Pose2d pose, Pose2d target) {
        return this.move(pose, target, null);
    }

    public boolean move(Pose2d pose, Pose2d target, Double[] power) {
        double x = driveX.getPower(target.getX(), pose.getX());
        double y = driveX.getPower(target.getY(), pose.getY());
        double h = driveX.getPower(target.getRotation().getDegrees(), pose.getRotation().getDegrees(), PID::rotationGetError);

        if (power != null) x *= power[0];
        if (power != null) y *= power[1];
        if (power != null) h *= power[2];

        drive.setDrivePowers(Drive.absoluteMovement(x, y, h, pose.getHeading()));

        boolean atX = PID.defaultGetError(target.getX(), pose.getX()) < this.tolX;
        boolean atY = PID.defaultGetError(target.getY(), pose.getY()) < this.tolY;
        boolean atH = PID.rotationGetError(target.getRotation().getDegrees(), pose.getRotation().getDegrees()) < this.tolH;

        return !atX && !atY && !atH;
    }
}