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
    public static final double TRACK_WIDTH = 15;
    public static final double CENTER_WHEEL_OFFSET = 0.5;
    public static final double WHEEL_DIAMETER = 35.0/25.4;
    public static final double TICKS_PER_REV = 8192.0;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    public static final double MAX_ACCEL = 33;
    public static final double MAX_VELOCITY = 100;

    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    private Pose2d pose;

    public Odometry(HardwareMap hardwareMap, IMU imu) {

        leftOdometer = new MotorEx(hardwareMap, "frontRight").encoder;
        rightOdometer = new MotorEx(hardwareMap, "frontLeft").encoder;
        centerOdometer = new MotorEx(hardwareMap, "backLeft").encoder;

        leftOdometer = leftOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = rightOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = centerOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );
        odometry.updatePose(new Pose2d(0, 0, new Rotation2d(0)));
        pose = odometry.getPose();
    }

    public void update() {
        odometry.updatePose();
        pose = odometry.getPose();
    }
    
    public Pose2d getPose() {
        return new Pose2d(pose.getY(), pose.getX(), pose.getRotation());
    }

    public double getX() {
        return pose.getY();
    }

    public double getY() {
        return pose.getX();
    }

    public double getHeading() {
        return pose.getRotation().getDegrees();
    }


    public double[] getEncoders() {
        return new double[]{ leftOdometer.getDistance(), rightOdometer.getDistance(), centerOdometer.getDistance() };
    }

    public void reset() {
        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();
        odometry.updatePose(new Pose2d(0, 0, new Rotation2d(0)));
        pose = odometry.getPose();
    }
}