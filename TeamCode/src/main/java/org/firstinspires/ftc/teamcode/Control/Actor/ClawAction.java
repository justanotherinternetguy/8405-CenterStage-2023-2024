package org.firstinspires.ftc.teamcode.Control.Actor;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.function.Predicate;

public class ClawAction extends Action {

    public enum ClawStates implements Predicate<ClawStates> {
        topOpen,
        topClosed,
        bottomOpen,
        bottomClosed;

        @Override
        public boolean test(ClawStates clawStates) {
            return clawStates == this;
        }
    }

    double tilt = 0;

    public ClawStates[] states = null;

    public ClawAction(ClawStates... states) {
        this.states = states;
    }

    public ClawAction(double tilt) { this.tilt = tilt; }

    public ClawAction(boolean isBackboard) {
        this(isBackboard ? 1.0 : 0);
    }

    @Override
    public void run(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        if (states == null) {
            double newPos = this.getNewTilt();
            robot.claw.clawServo.setPosition(newPos);
            tm.addData("tiltpos", newPos);
            return;
        }
        for (ClawStates state : states) {
            switch (state) {
                case topOpen:
                    robot.claw.topServo.setPosition(Config.topServoOpen);
                    break;
                case topClosed:
                    robot.claw.topServo.setPosition(Config.topServoClose);
                    break;
                case bottomOpen:
                    robot.claw.bottomServo.setPosition(Config.bottomServoOpen);
                    break;
                case bottomClosed:
                    robot.claw.bottomServo.setPosition(Config.bottomServoClose);
                    break;
            }
        }
    }

    public double getNewTilt() {
        double range = Config.clawServoBackboard - Config.clawServoFloor;
        double offset = range * this.tilt;
        return Config.clawServoFloor + offset;
    }

    @Override
    public boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        // use timeout to determine(servo cannot teleport instantly)
        return false;
    }

    @Override
    public double defaultTimeout(Pose2d pose, int lift, Double prevTilt, ClawStates[] prevClaw) {
        return 2000;
//        if (states == null) {
////            double distance = prevTilt - tilt;
////            return distance * 2000;
//            return 2000;
//        }
//        double maxTime = 0;
//        for (ClawStates state : prevClaw) {
//            switch (state) {
//                case topOpen:
//                    if (Arrays.stream(states).anyMatch(ClawStates.topClosed)) maxTime = Math.max(maxTime, 750);
//                    break;
//                case topClosed:
//                    if (Arrays.stream(states).anyMatch(ClawStates.topOpen)) maxTime = Math.max(maxTime, 750);
//                    break;
//                case bottomOpen:
//                    if (Arrays.stream(states).anyMatch(ClawStates.bottomClosed)) maxTime = 1500;
//                    break;
//                case bottomClosed:
//                    if (Arrays.stream(states).anyMatch(ClawStates.bottomOpen)) maxTime = 1500;
//                    break;
//            }
//        }
//        return maxTime;
    }
}
