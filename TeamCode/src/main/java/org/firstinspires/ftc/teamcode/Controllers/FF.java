package org.firstinspires.ftc.teamcode.Controllers;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.teamcode.utils;


public class FF {
    MecanumDriveKinematics kinematics;
    double max_motor_speed;

    public FF(double track_width, double wheelbase_length, double max_motor_speed) {
        Translation2d frontLeft = new Translation2d(wheelbase_length / 2, track_width / 2);
        Translation2d frontRight = new Translation2d(wheelbase_length / 2, -track_width / 2);
        Translation2d backLeft = new Translation2d(-wheelbase_length / 2, track_width / 2);
        Translation2d backRight = new Translation2d(-wheelbase_length / 2, -track_width / 2);
        this.kinematics = new MecanumDriveKinematics(frontLeft, frontRight, backLeft, backRight);
        this.max_motor_speed = max_motor_speed;

    }

    private static ChassisSpeeds findNextVector(double cur_x, double cur_y, double cur_theta,
                                                double to_x, double to_y, double to_theta) {
        double x_comp = to_x - cur_x;
        double y_comp = to_y - cur_y;
        double theta_comp = utils.angleDifference(cur_theta, to_theta);
        return new ChassisSpeeds(x_comp, y_comp, theta_comp);

    }

    public MecanumDriveWheelSpeeds getSpeedsForPoint(double cur_x, double cur_y, double cur_theta,
                                                     double to_x, double to_y, double to_theta) {
        MecanumDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(findNextVector(cur_x, cur_y, cur_theta, to_x, to_y, to_theta));
        double[] speeds_ = new double[]{speeds.frontLeftMetersPerSecond,
                speeds.frontRightMetersPerSecond,
                speeds.rearLeftMetersPerSecond,
                speeds.rearRightMetersPerSecond};
        double normalizeTo = -1;
        for (double speed : speeds_) {
            if (speed > this.max_motor_speed) {
                normalizeTo = Math.max(normalizeTo, speed);
            }
        }
        if (normalizeTo != -1) {
            for (int i = 0; i < speeds_.length; i++) {
                speeds_[i] = speeds_[i] / normalizeTo;
            }
        }
        return new MecanumDriveWheelSpeeds(speeds_[0], speeds_[1], speeds_[2], speeds_[3]);

    }
}
