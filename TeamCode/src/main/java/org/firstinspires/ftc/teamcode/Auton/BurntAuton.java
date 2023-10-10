package org.firstinspires.ftc.teamcode.Auton;

// Overcooked the Auton in burnt
// This auton is basically just a POC made by abusing FTCLib functions
// IDK if it'll actually work until tuesday when we can test it on the bot

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;


@Autonomous
public class BurntAuton extends LinearOpMode {
    Drive drive;
    Odometry odometry;
    Pose2d targetPose;
    ElapsedTime timer;
    double lastMS;
    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap);
        odometry = new Odometry(hardwareMap, drive.imu);
        targetPose = new Pose2d(0, 24, new Rotation2d(Math.toRadians(45)));
        timer = new ElapsedTime();

        waitForStart();

        odometry.reset();
        drive.imu.resetYaw();
        Pose2d initialOffset = new Pose2d(this.getRawX(), this.getRawY(), new Rotation2d(this.getRawHeading()));
        lastMS = timer.milliseconds();
        double lastIMU = drive.getIMU();

        while (opModeIsActive() && !this.atTarget()) {
            odometry.update();

            double newIMU = drive.getIMU();
            if (newIMU == lastIMU) {
                // imu hasn't updated yet
                // predict the next imu value
                double delta = timer.milliseconds() - lastMS;
                double nextHeading = lastIMU + drive.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * delta / 1000;
                telemetry.addData("NEXT", nextHeading);
                lastMS = timer.milliseconds();
            } else {
                lastIMU = newIMU;
            }

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(initialOffset.getX(), initialOffset.getY(), initialOffset.getHeading(), odometry.getPose().getRotation());

            double rotX = speeds.vxMetersPerSecond * 1.1;
            double rotY = speeds.vyMetersPerSecond;
            double rx = speeds.omegaRadiansPerSecond;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            telemetry.addData("Chassis Speeds ", speeds.toString());
            telemetry.addData("Odometry Pose ", odometry.getPose().toString());
            telemetry.addData("IMU Heading ", Math.toDegrees(drive.getIMU()));
            telemetry.addData("Heading Drift", Math.toDegrees(drive.getIMU() - odometry.getPose().getHeading()));
            telemetry.addData("Target Pose ", targetPose.toString());
            telemetry.addData("Initial Offset ", initialOffset.toString());
            telemetry.addData("Delta ", timer.milliseconds() - lastMS);
            telemetry.addData("Angular Velocity ", drive.imu.getRobotAngularVelocity(AngleUnit.DEGREES));
            telemetry.update();

            drive.setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }
        drive.setDrivePowers(0, 0, 0, 0);
    }

    public boolean atTarget() {
        double translationTolerance = 0.5;
        double headingTolerance = 2;

        boolean xAtTarget = Math.abs(getRawX()) < translationTolerance;
        boolean yAtTarget = Math.abs(getRawY()) < translationTolerance;
        boolean headingAtTarget = Math.abs(getRawHeading()) < headingTolerance;

        return xAtTarget && yAtTarget && headingAtTarget;
    }

    public double getRawX() {
        return targetPose.getX() - odometry.getPose().getX();
    }

    public double getRawY() {
        return targetPose.getY() - odometry.getPose().getY();
    }

    public double getRawHeading() {
        return targetPose.getHeading() - odometry.getPose().getHeading();
    }
}
