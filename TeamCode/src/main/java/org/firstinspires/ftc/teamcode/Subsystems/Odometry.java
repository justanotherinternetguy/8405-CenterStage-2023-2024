package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.utils.normalizeRadians;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Slimmed down version of VirtualRobot EncBot odom for temporary use
public class Odometry {
    private DcMotorEx[] encoders = new DcMotorEx[3];
    private final double ENCODER_TICKS_PER_REVOLUTION = 1120;
    private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * 2.0;
    private final double ENCODER_WIDTH = 12.0;
    private int[] prevTicks = new int[3];

    public double x;
    public double y;
    public double heading;

    public Odometry(HardwareMap hwMap) {
        String[] encoderNames = new String[]{"enc_right", "enc_left", "enc_x"};
        for (int i = 0; i < 3; i++) encoders[i] = hwMap.get(DcMotorEx.class, encoderNames[i]);

        this.x = 0;
        this.y = 0;
        this.heading = 0;
    }

    public double[] update() {
        int[] ticks = new int[3];
        for (int i = 0; i < 3; i++) ticks[i] = encoders[i].getCurrentPosition();
        int newRightTicks = ticks[0] - prevTicks[0];
        int newLeftTicks = ticks[1] - prevTicks[1];
        int newXTicks = ticks[2] - prevTicks[2];
        prevTicks = ticks;
        double rightDist = newRightTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double leftDist = -newLeftTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double dyR = 0.5 * (rightDist + leftDist);
        double headingChangeRadians = (rightDist - leftDist) / ENCODER_WIDTH;
        double dxR = -newXTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double avgHeadingRadians = Math.toRadians(-heading) + headingChangeRadians / 2.0;
        double cos = Math.cos(avgHeadingRadians);
        double sin = Math.sin(avgHeadingRadians);
        y += dxR * sin + dyR * cos;
        x += dxR * cos - dyR * sin;
        heading = -Math.toDegrees(normalizeRadians(Math.toRadians(-heading) + headingChangeRadians));
        return new double[]{x, y, heading};
    }
}