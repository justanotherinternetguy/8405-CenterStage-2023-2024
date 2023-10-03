package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Subsystems.*;

/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */
public class Odometry {
    public static final double TRACKWIDTH = 13.25;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = 0.0;

    public static final double WHEEL_DIAMETER = 35.0/25.4;
    public static final double TICKS_PER_REV = 8192.0;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    private Pose2d pose;

    public Odometry(HardwareMap hardwareMap) {
        leftOdometer = leftOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = rightOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = centerOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        odometry.updatePose(); // update the position
        Pose2d pose = odometry.getPose();
    }

    public void update() {
        odometry.updatePose();
        pose = odometry.getPose();
    }
    
    public Pose2d getPose() {
        return pose;
    }

    public double getX() {
        return pose.getX();
    }

    public double getY() {
        return pose.getY();
    }

    public double getHeading() {
        return pose.getHeading();
    }

    public double[] getEncoders() {
        return new double[]{ leftOdometer.getDistance(), rightOdometer.getDistance(), centerOdometer.getDistance() };
    }
}