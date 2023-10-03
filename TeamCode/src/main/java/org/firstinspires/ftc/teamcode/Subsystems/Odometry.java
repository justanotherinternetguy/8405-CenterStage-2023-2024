package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.utils.normalizeRadians;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Base64;

// Slimmed down version of VirtualRobot EncBot odom for temporary use
public class Odometry {
    public double x = 0.0;
    public double y = 0.0;
    public double heading = 0.0;

    public double prev_left_pos = 0.0;
    public double prev_right_pos = 0.0;
    public double prev_perp_pos = 0.0;

    static final double TRACKWIDTH = 13.25;
    static final double WHEEL_DIAMETER = 35.0;
    static final double CENTER_WHEEL_OFFSET = 0;
    static final double TICKS_PER_REVOLUTION = 8192.0;
    static final double mm_to_inches = 0.0393700787;
    static final double wheel_circumference_mm = 2 * Math.PI * 17.5;
    static final double wheel_circumference_inches = (2 * Math.PI * 17.5)/25.4;
    static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REVOLUTION;

    public static double getCenterWheelOffset() {
        return CENTER_WHEEL_OFFSET;
    }

    private MotorEx enc_left, enc_right, enc_perp;

    public Odometry(MotorEx get_enc_left, MotorEx get_enc_right, MotorEx get_enc_perp) {
        enc_left = get_enc_left;
        enc_right = get_enc_right;
        enc_perp = get_enc_perp;

    }
}