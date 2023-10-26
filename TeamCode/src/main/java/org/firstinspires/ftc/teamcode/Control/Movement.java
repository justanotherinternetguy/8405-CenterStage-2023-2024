package org.firstinspires.ftc.teamcode.Control;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils;

import java.util.Arrays;
import java.util.function.Supplier;

public class Movement {
    private Drive drive;
    private SampleMecanumDrive rrDrive;
    private PID xPID;
    private PID yPID;
    private PID hPID;
    private double tolerance;
    private Telemetry telemetry;
    public enum DIRECTION{LEFT, RIGHT};
    private ElapsedTime timer;

    public Movement(Drive drive, SampleMecanumDrive rrDrive, PID.Config drivePIDConfig, PID.Config headingPIDConfig, double tolerance, Telemetry telemetry) {
        this.drive = drive;
        this.rrDrive = rrDrive;
        this.xPID = new PID(drivePIDConfig);
        this.yPID = new PID(drivePIDConfig);
        this.hPID = new PID(headingPIDConfig);
        this.tolerance = tolerance;
        this.telemetry = telemetry;

        timer = new ElapsedTime();
    }
    public boolean goTo(Pose2d target) {
        Pose2d pose = rrDrive.getPose();
        double xTolerance = 1;
        double yTolerance = 1;
        double hTolerance = 2;

        double accelMax = Math.min(timer.milliseconds() / 750, 1);
//        double accelMax = 1;
        System.out.println(accelMax + " " + timer.seconds());

        telemetry.addData("max", accelMax);

        double x = xPID.getError(target.getX(), pose.getX(), accelMax);
        double y = yPID.getError(target.getY(), pose.getY(), accelMax);
        double rx = hPID.getError(target.getHeading(), pose.getHeading(), accelMax);

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);
        fieldCentricMove(x, y, rx, accelMax);

        return Math.abs(target.getX() - pose.getX()) < xTolerance && Math.abs(target.getY() - pose.getY()) < yTolerance && Math.abs(PID.angleWrap(Math.toDegrees(target.getHeading() - pose.getHeading()))) < hTolerance;
    }
    public void fieldCentricMove(Pose2d dir) {
        fieldCentricMove(dir, 1);
    }
    public void fieldCentricMove(Pose2d dir, double maxPower) {
        fieldCentricMove(dir.getX(), dir.getY(), dir.getHeading(), maxPower);
    }
    public void fieldCentricMove(double x, double y, double rx, double maxPower) {
        double botHeading = -rrDrive.getPose().getRotation().getRadians();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1) * 1 / maxPower;
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        drive.frontLeft.setPower(frontLeftPower);
        drive.backLeft.setPower(backLeftPower);
        drive.frontRight.setPower(frontRightPower);
        drive.backRight.setPower(backRightPower);
    }
}