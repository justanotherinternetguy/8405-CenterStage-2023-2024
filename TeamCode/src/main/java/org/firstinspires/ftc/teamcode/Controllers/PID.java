package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private PID.Config settings;

    private double integralSum = 0.0;
    private double lastError = 0.0;

    private ElapsedTime timer;

    public PID(Config settings) {
        this.settings = settings;
        timer = new ElapsedTime();
        timer.reset();
    }

    public PID(double Kp, double Ki, double Kd) {
        this.settings = new PID.Config(Kp, Ki, Kd);
        timer = new ElapsedTime();
        timer.reset();
    }

    // error is given by the user so that this class has multiple use cases
    public double getValue(double error) {
        double dT = timer.seconds();
        double derivative = (error - lastError) / dT;

        // sum all the error over time
        integralSum += (error * dT);

        // returns this value which can differ depending on the use case of the class
        // eg. set motor power
        double out = (settings.getKp() * error) + (settings.getKi() * integralSum) + (settings.getKd() * derivative);

        lastError = error;

        timer.reset();

        return out;
    }

    public void reset() {
        lastError = 0;
        integralSum = 0;
        timer.reset();
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
    }
}