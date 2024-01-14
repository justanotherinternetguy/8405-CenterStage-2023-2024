package org.firstinspires.ftc.teamcode.Control.Actor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class ClawAction extends Action {
    public enum ClawStates {
        topOpen,
        topClosed,
        bottomOpen,
        bottomClosed
    }

    boolean isBackboard = false;

    private ClawStates[] states = null;

    public ClawAction(ClawStates... states) {
        this.states = states;
    }

    public ClawAction(boolean isBackboard) {
        this.isBackboard = isBackboard;
    }

    @Override
    public void run(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        if (states == null) {
            if (isBackboard) {
                robot.claw.clawServo.getController().pwmEnable();
                robot.claw.clawServo.setPosition(Config.clawServoBackboard);
            } else {
//            clawServo.setPosition(Config.clawServoFloor);
                robot.claw.clawServo.getController().pwmDisable();
            }
//            if (isBackboard) {
//                robot.claw.clawServo.setPosition(Config.clawServoBackboard);
//            } else {
//                robot.claw.clawServo.setPosition(Config.clawServoFloor);
//            }
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

    @Override
    public boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        // use timeout to determine(servo cannot teleport instantly)
        return false;
    }
}
