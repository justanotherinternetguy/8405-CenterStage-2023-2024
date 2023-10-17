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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils;

import java.util.function.Supplier;

public class Movement {
    private Drive drive;
    private SampleMecanumDrive rrDrive;
    private PID driveXPID;
    private PID driveYPID;
    private PID headingPID;
    private Supplier<Boolean> opModeIsActive;
    private double tolerance;
    private Telemetry telemetry;

    private ElapsedTime timer;

    public Movement(Drive drive, SampleMecanumDrive rrDrive, Supplier<Boolean> opModeIsActive, PID.Config drivePIDConfig, PID.Config headingPIDConfig, double tolerance, Telemetry telemetry) {
        this.drive = drive;
        this.rrDrive = rrDrive;
        this.driveXPID = new PID(drivePIDConfig);
        this.driveYPID = new PID(drivePIDConfig);
        this.headingPID = new PID(headingPIDConfig);
        this.opModeIsActive = opModeIsActive;
        this.tolerance = tolerance;
        this.telemetry = telemetry;

        timer = new ElapsedTime();
    }

    public void move(Pose2d target) {
        Pose2d init = rrDrive.getPose();
        Pose2d init_target_pose = new Pose2d(target.getX() - init.getX(), target.getY() - init.getY(), new Rotation2d(Math.toRadians(utils.angleDifference(target.getRotation().getDegrees(), init.getRotation().getDegrees()))));
        timer.reset();
        double elapsed_time;
        Pose2d pose = init;
//        double lastMS = timer.milliseconds();
//        double lastIMU = drive.getIMU();

        while (opModeIsActive.get() && (Math.abs(target.getX() - pose.getX()) > tolerance || Math.abs(target.getY() - pose.getY()) > tolerance || Math.abs(utils.angleDifference(target.getRotation().getDegrees(), pose.getRotation().getDegrees())) > 3.0)) {
            elapsed_time = timer.seconds();
            rrDrive.update();
            pose = rrDrive.getPose();
            //
            double instantTargetPositionX = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, init_target_pose.getX(), elapsed_time) + init.getX();
            double instantTargetPositionY = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, init_target_pose.getY(), elapsed_time) + init.getY(); // (-90 - 90) + 90 = -180 + 90 = -90
            double instantTargetPositionH = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, init_target_pose.getRotation().getDegrees(), elapsed_time)  + Math.toDegrees(init.getHeading());
            //
            double x = driveXPID.getValue(instantTargetPositionX - pose.getX());
            double y = driveYPID.getValue(instantTargetPositionY - pose.getY());
//            double rx = Math.toRadians(headingPID.getValue(instantTargetPositionH - odom.getHeading()));
//            double x = driveXPID.getValue(target.getX() - pose.getX());
//            double y = driveYPID.getValue(target.getY() - pose.getY());
//            double rx = headingPID.getValue(target.getRotation().getDegrees() - odom.getHeading());
            double rx = headingPID.getValue(utils.angleDifference(target.getRotation().getDegrees(), Math.toDegrees(pose.getHeading())));
            // pid <- angleDiff(targ.deg, deg <- heading))
//            double botHeading = Math.toRadians(odom.getHeading());
//            double imuValue = drive.getIMU();

//            double delta = timer.milliseconds() - lastMS;
//            double nextHeading = lastIMU + drive.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * delta / 1000;
//            double botHeading = Math.toRadians(lastIMU == imuValue ? nextHeading : imuValue);
            double botHeading = pose.getHeading();

//            double x = target.getX() - pose.getX();
//            double y = target.getY() - pose.getY();
//            double rx = 0;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

//            telemetry.addData("X motion ", instantTargetPositionX);
//            telemetry.addData("Y motion ", instantTargetPositionY);
//            telemetry.addData("H motion ", instantTargetPositionH);;
//            telemetry.addData("X init ", init_target_pose.getX());
//            telemetry.addData("Y init ", init_target_pose.getY());
//            telemetry.addData("H init ", init_target_pose.getRotation().getDegrees());
//            telemetry.addData("Elapsed Time ", elapsed_time);
//            telemetry.addData("Pose ", pose);
//            telemetry.addData("XXXX ", x);
//            telemetry.addData("YYYY ", y);
//            telemetry.addData("x Error ", x);
//            telemetry.addData("y Error ", y);
            telemetry.addData("atX ", Math.abs(target.getX() - pose.getX()) > tolerance);
            telemetry.addData("atY ", Math.abs(target.getY() - pose.getY()) > tolerance);
            telemetry.addData("atH ", Math.abs(utils.angleDifference(target.getRotation().getDegrees(), pose.getRotation().getDegrees())) > 3.0);
            telemetry.addData("x", driveXPID.settings);
            telemetry.addData("y", driveXPID.settings);
            telemetry.addData("h", driveXPID.settings);
//            telemetry.addData("h Error ", rx);
//            telemetry.addData("raw hE", utils.angleDifference(target.getRotation().getDegrees(), Math.toDegrees(pose.getHeading())));
//            telemetry.addData("raw hE2", utils.angleDifference(target.getRotation().getDegrees(), pose.getRotation().getDegrees()));
//            telemetry.addData("subtract", target.getRotation().getDegrees() - Math.toDegrees(pose.getHeading()));
//            telemetry.addData("subtract2", target.getRotation().getDegrees() - pose.getRotation().getDegrees());
//            telemetry.addData("targ", target.getRotation().getDegrees());
//            telemetry.addData("rad", pose.getHeading());
//            telemetry.addData("botHeading", Math.toDegrees(botHeading));
//            telemetry.addData("botHeading", pose.getRotation().getDegrees());
//            telemetry.addData("rotX  ", rotX);
//            telemetry.addData("rotY  ", rotY);
//            telemetry.addData("FL ", frontLeftPower);
//            telemetry.addData("FR ", frontRightPower);
//            telemetry.addData("BL ", backLeftPower);
//            telemetry.addData("BR ", backRightPower);
            telemetry.update();

            drive.setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }
        drive.setDrivePowers(0,0,0,0);
    }
}