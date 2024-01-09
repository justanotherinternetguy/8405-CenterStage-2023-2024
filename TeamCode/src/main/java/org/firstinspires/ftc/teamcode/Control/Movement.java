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

    public boolean move(Pose2d pose, Pose2d target, Telemetry tel) {
        return this.move(pose, target, null, tel);
    }

    public boolean move(Pose2d pose, Pose2d target, Double[] power, Telemetry tel) {
        double x = driveX.getPower(target.getX(), pose.getX());
        double y = driveY.getPower(target.getY(), pose.getY());
        double h = driveH.getPower(target.getRotation().getDegrees(), pose.getRotation().getDegrees(), PID::rotationGetError);

        if (power != null) x *= power[0];
        if (power != null) y *= power[1];
        if (power != null) h *= power[2];

        Drive.DrivePowers powers = Drive.absoluteMovement(x, y, h, pose.getHeading());
        drive.setDrivePowers(powers);

        double atX = Math.abs(PID.defaultGetError(target.getX(), pose.getX()));
        double atY = Math.abs(PID.defaultGetError(target.getY(), pose.getY()));
        double atH = Math.abs(PID.rotationGetError(target.getRotation().getDegrees(), pose.getRotation().getDegrees()));

        if (tel != null) {
            tel.addData("atX", atX);
            tel.addData("atY", atY);
            tel.addData("atH", atH);
            tel.addData("eX", x);
            tel.addData("eY", y);
            tel.addData("eH", h);
            tel.addData("eX2", target.getX() - pose.getX());
            tel.addData("eY2", target.getY() - pose.getY());
            tel.addData("eH2", PID.rotationGetError(target.getRotation().getDegrees(), pose.getRotation().getDegrees()));
            tel.addData("x", pose.getX());
            tel.addData("y", pose.getY());
            tel.addData("h", pose.getRotation().getDegrees());
            tel.addData("fl", powers.frontLeft);
            tel.addData("fr", powers.frontRight);
            tel.addData("bl", powers.backLeft);
            tel.addData("br", powers.backRight);
            tel.update();
        }

        return !(atX < this.tolX) || !(atY < this.tolY) || !(atH < this.tolH);
    }
}