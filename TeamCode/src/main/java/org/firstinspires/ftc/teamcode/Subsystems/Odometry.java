package org.firstinspires.ftc.teamcode.Subsystems;

import android.provider.ContactsContract;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */
@NonNull
public class Odometry {
    // TODO: Move to bot constants
    public static final double TRACKWIDTH = 15;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = 0.5;

    public static final double WHEEL_DIAMETER = 35.0/25.4;
    public static final double TICKS_PER_REV = 8192.0;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    public static final double MAX_ACCEL = 50;
    public static final double MAX_VELO = 50;

    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    private Pose2d pose;
    private IMU imu;
    public Odometry(HardwareMap hardwareMap, IMU imu) {
        this.imu= imu;
//        leftOdometer = hardwareMap.get(MotorEx.class, "frontRight").encoder;
//        rightOdometer = hardwareMap.get(MotorEx.class, "frontLeft").encoder;
//        centerOdometer = hardwareMap.get(MotorEx.class, "backLeft").encoder;

        leftOdometer = new MotorEx(hardwareMap, "frontRight").encoder;
        rightOdometer = new MotorEx(hardwareMap, "frontLeft").encoder;
        centerOdometer = new MotorEx(hardwareMap, "backLeft").encoder;

        leftOdometer = leftOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = rightOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = centerOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);

//        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        odometry.updatePose(new Pose2d(0, 0, new Rotation2d(0))); // update the position
        pose = odometry.getPose();
    }

    public void update() {
        odometry.updatePose();
        pose = odometry.getPose();
    }
    
    public Pose2d getPose() {
        return new Pose2d(pose.getY(), pose.getX(), pose.getRotation());
//        return new Pose2d(pose.getY(), pose.getX(), new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
    }

    // CHANGE THIS LATER~!!!!!!!!!!!

    public double getX() {
        return pose.getY();
    }

    public double getY() {
        return pose.getX();
    }

    public double getHeading() {
        return pose.getRotation().getDegrees();
//        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    public double[] getEncoders() {
        return new double[]{ leftOdometer.getDistance(), rightOdometer.getDistance(), centerOdometer.getDistance() };
    }

    public void reset() {
        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();
        odometry.updatePose(new Pose2d(0, 0, new Rotation2d(0))); // update the position
        pose = odometry.getPose();
    }

    public void sync(double imuHeading, Telemetry t) {
//        this.update();
//        Pose2d posey = new Pose2d(pose.getY(), pose.getX(), new Rotation2d(Math.toRadians(imuHeading)));
//        t.addData("HEADING!!!!! ", posey);
//        odometry.updatePose(posey);
    }
}