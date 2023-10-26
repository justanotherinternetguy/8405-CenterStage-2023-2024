package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double Kp;
    private double Ki;
    private double Kd;
    private double maxIntegralSum;
    private double a;
    private boolean angleSupport;

    private Double lastTarget;
    private double integralSum;
    private double lastError;
    private double previousFilterEstimate;
    ElapsedTime timer = new ElapsedTime();

    public PID(Config config) {
        this(config, false);
    }

    public PID(Config config, boolean angleSupport) {
        this(config.getKp(), config.getKi(), config.getKd(), 0, angleSupport);
    }

    public PID(double Kp, double Ki, double Kd, double a) { // a = 0.8; // a can be anything from 0 < a < 1
        this(Kp, Ki, Kd, a, 0.25 / Ki, false);
    }
    public PID(double Kp, double Ki, double Kd, double a, boolean angleSupport) {
        this(Kp, Ki, Kd, a, 0.25 / Ki, angleSupport);
    }
    public PID(double Kp, double Ki, double Kd, double a, double maxIntegralSum) {
        this(Kp, Ki, Kd, a, maxIntegralSum, false);
    }
    public PID(double Kp, double Ki, double Kd, double a, double maxIntegralSum, boolean angleSupport) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.a = a;
        this.maxIntegralSum = maxIntegralSum;
        this.angleSupport = angleSupport;
        this.reset();
    }

    public void reset() {
        lastTarget = null;
        integralSum = 0;
        lastError = 0;
        previousFilterEstimate = 0;
        timer.reset();
    }

    public double getError(double target, double current) {
        return getError(target, current, 1);
    }

    public double getError(double target, double current, double accelMax) {
        if (lastTarget == null) lastTarget = target;

        // calculate the error
        double error = target - current;
        if (angleSupport) {
            error = angleWrap(error);
        }

        double errorChange = (error - lastError);

        // filter out height frequency noise to increase derivative performance
        double currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // rate of change of the error
        double derivative = currentFilterEstimate / timer.seconds() / accelMax;

        // sum of all error over time
        integralSum += (error * timer.seconds()) * accelMax;

        // max out integral sum
        if (integralSum > maxIntegralSum) {
            integralSum = maxIntegralSum;
        }
        if (integralSum < -maxIntegralSum) {
            integralSum = -maxIntegralSum;
        }

        // reset integral sum upon target changes
        if (target != lastTarget) {
            integralSum = 0;
        }

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;
        lastTarget = target;

        timer.reset();

        return out;
    }

    // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
    public static double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
    public static class Config {
        private double Kp;
        private double Ki;
        private double Kd;

        public Config(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public double getKp() {
            return Kp;
        }

        public double getKi() {
            return Ki;
        }

        public double getKd() {
            return Kd;
        }

        @Override
        public String toString() {
            return "{ Kp: " + getKp() + ", Ki: " + getKi() + ", Kd: " + getKd() + " }";
        }
    }
}