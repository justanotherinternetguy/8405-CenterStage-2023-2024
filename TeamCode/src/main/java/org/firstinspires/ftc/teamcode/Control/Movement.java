package org.firstinspires.ftc.teamcode.Control;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.utils;

import java.util.function.Supplier;

public class Movement {
    private Drive drive;
    private Odometry odom;
    private PID driveXPID;
    private PID driveYPID;
    private PID headingPID;
    private Supplier<Boolean> opModeIsActive;
    private double tolerance;
    private Telemetry telemetry;

    private ElapsedTime timer;

    public Movement(Drive drive, Odometry odom, Supplier<Boolean> opModeIsActive, PID.Config drivePIDConfig, PID.Config headingPIDConfig, double tolerance, Telemetry telemetry) {
        this.drive = drive;
        this.odom = odom;
        this.driveXPID = new PID(drivePIDConfig);
        this.driveYPID = new PID(drivePIDConfig);
        this.headingPID = new PID(headingPIDConfig);
        this.opModeIsActive = opModeIsActive;
        this.tolerance = tolerance;
        this.telemetry = telemetry;

        timer = new ElapsedTime();
    }

    public void move(Pose2d target) {
        // 0 - 24 = -24 | S = 24 T = 0
        // 24 - 0 = 24 | S = 0 T = 24
        Pose2d init = odom.getPose();
        Pose2d init_pose = new Pose2d(target.getX() - odom.getX(), target.getY() - odom.getY(), new Rotation2d(Math.toRadians(target.getRotation().getDegrees() - odom.getHeading())));
        timer.reset();
        double elapsed_time;
        double lastMS = timer.milliseconds();
        double lastIMU = drive.getIMU();

        while (opModeIsActive.get() && (Math.abs(target.getX() - odom.getX()) > tolerance || Math.abs(target.getY() - odom.getY()) > tolerance || Math.abs(target.getRotation().getDegrees() - odom.getHeading()) > tolerance * 3)) {
            elapsed_time = timer.seconds();
            odom.update();
            //
            double instantTargetPositionX = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, init_pose.getX(), elapsed_time) + init.getX();
            double instantTargetPositionY = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, init_pose.getY(), elapsed_time) + init.getY(); // (-90 - 90) + 90 = -180 + 90 = -90
            double instantTargetPositionH = MotionProfile.motion_profile(Odometry.MAX_ACCEL * 4, Odometry.MAX_VELOCITY * 4, init_pose.getRotation().getDegrees(), elapsed_time)  + init.getRotation().getDegrees();
            //
//            double x = driveXPID.getValue(instantTargetPositionX - odom.getX());
//            double y = driveYPID.getValue(instantTargetPositionY - odom.getY());
//            double rx = Math.toRadians(headingPID.getValue(instantTargetPositionH - odom.getHeading()));
            double x = driveXPID.getValue(target.getX() - odom.getX());
            double y = driveYPID.getValue(target.getY() - odom.getY());
//            double rx = headingPID.getValue(target.getRotation().getDegrees() - odom.getHeading());
            double rx = headingPID.getValue(utils.angleDifference(target.getRotation().getDegrees(), odom.getHeading()));
//            double botHeading = Math.toRadians(odom.getHeading());
            double imuValue = drive.getIMU();

            double delta = timer.milliseconds() - lastMS;
            double nextHeading = lastIMU + drive.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * delta / 1000;
            double botHeading = Math.toRadians(lastIMU == imuValue ? nextHeading : imuValue);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1) * 3;
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

//            telemetry.addData("X motion ", instantTargetPositionX);
//            telemetry.addData("Y motion ", instantTargetPositionY);
            telemetry.addData("H motion ", instantTargetPositionH);;
//            telemetry.addData("X init ", init_pose.getX());
//            telemetry.addData("Y init ", init_pose.getY());
            telemetry.addData("H init ", init_pose.getRotation().getDegrees());
            telemetry.addData("Elapsed Time ", elapsed_time);
            telemetry.addData("Pose ", odom.getPose().toString());
//            telemetry.addData("x Error ", x);
//            telemetry.addData("y Error ", y);
            telemetry.addData("h Error ", rx);
            telemetry.addData("botHeading", Math.toDegrees(botHeading));
            telemetry.addData("rotX  ", rotX);
            telemetry.addData("rotY  ", rotY);
            telemetry.addData("FL ", frontLeftPower);
            telemetry.addData("FR ", frontRightPower);
            telemetry.addData("BL ", backLeftPower);
            telemetry.addData("BR ", backRightPower);
            telemetry.update();

            drive.setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }
        drive.setDrivePowers(0,0,0,0);
    }
}