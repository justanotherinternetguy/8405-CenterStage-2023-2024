package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils;

import java.util.function.DoubleBinaryOperator;

public class PID {
    /*
     * Proportional Integral Derivative Controller w/ Low pass filter and anti-windup
     */

    private Config config;

    Double lastTarget = null;
    double integralSum = 0;
    double lastError = 0;
    double maxIntegralSum = Double.MAX_VALUE;
    double a = 0.8; // a can be anything from 0 < a < 1
    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;

    // Elapsed timer class from SDK, please use it, it's epic
    ElapsedTime timer = null;

    public PID(Config config) {
        this.config = config;
        this.maxIntegralSum = config.i / 0.25;
    }

    public PID(double p, double i, double d) {
        this(new Config(p, i, d));
    }

    public double getPower(double target, double current) {
        return this.getPower(target, current, PID::defaultGetError);
    }

    public static double defaultGetError(double target, double current) {
       return target - current;
    }

    public static double rotationGetError(double target, double current) {
        return utils.angleDifference(target, current);
    }

    public double getPower(double target, double current, DoubleBinaryOperator getError) {
        // on the first run, we will initalize lastTarget and timer
        if (lastTarget == null) lastTarget = target;
        if (timer == null) {
            timer = new ElapsedTime();
            timer.reset();
        }

        // calculate the error
        double error = getError.applyAsDouble(target, current);

        double errorChange = (error - lastError);

        // filter out hight frequency noise to increase derivative performance
        currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // rate of change of the error
        double derivative = currentFilterEstimate / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());


        // max out integral sum
        if (integralSum > maxIntegralSum) {
            integralSum = maxIntegralSum;
        }

        if (integralSum < -maxIntegralSum) {
            integralSum = -maxIntegralSum;
        }

        // reset integral sum upon setpoint changes
        if (target != lastTarget) {
            integralSum = 0;
        }

        double power = (config.getP() * error) + (config.getI() * integralSum) + (config.getD() * derivative);

        lastError = error;
        lastTarget = target;
        timer.reset();

        return power;
    }

    public static class Config {
        private double p;
        private double i;
        private double d;

        public Config(double p, double i, double d) {
            this.p = p;
            this.i = i;
            this.d = d;
        }

        public double getP() { return p; }
        public double getKp() { return this.getP(); }

        public void setP(double p) { this.p = p; }
        public void setKp(double p) { this.setP(p); }

        public double getI() { return i; }
        public double getKi() { return this.getI(); }

        public void setI(double i) { this.i = i; }
        public void setKi(double i) { this.setI(i); }

        public double getD() { return d; }
        public double getKd() { return this.getD(); }

        public void setD(double d) { this.d = d; }
        public void setKd(double d) { this.setD(d); }
    }
}