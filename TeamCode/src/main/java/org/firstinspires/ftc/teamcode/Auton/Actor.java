package org.firstinspires.ftc.teamcode.Auton;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;

public class Actor {
    private final HardwareMap hw;
    private final Telemetry tm;
    private final Robot robot;
    private final SampleMecanumDrive rrDrive;
    private final Movement movement;

    private final ElapsedTime timer = new ElapsedTime();

    private final double defaultTimeout;

    public Actor(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement, double defaultTimeout) {
        this.hw = hw;
        this.tm = tm;
        this.robot = robot;
        this.rrDrive = rrDrive;
        this.movement = movement;
        this.defaultTimeout = defaultTimeout;
    }

    public ArrayList<ArrayList<Action>> actions = new ArrayList<>();

    public Actor add(Action action, Double timeout, boolean parallel, boolean perpetual) {
        action.perpetual = perpetual; // this way both are in the add params not the action params
        action.timeout = timeout != null ? timeout : defaultTimeout;
        if (parallel) {
            actions.get(actions.size() - 1).add(action);
        } else {
            actions.add(new ArrayList<>());
            actions.get(actions.size() - 1).add(action);
        }
        return this;
    }

    public Actor add(Action action, boolean parallel, boolean perpetual) {
        return this.add(action, null, parallel, perpetual);
    }

    public Actor add(Action action, boolean parallel) {
        return this.add(action, null,  parallel, false);
    }

    public Actor add(Action action, Double timeout, boolean parallel) {
        return this.add(action, timeout, parallel, false);
    }

    public Actor add(Action action, Double timeout) {
        return this.add(action, timeout, false, false);
    }

    public Actor add(Action action) {
        return this.add(action, null);
    }

    public void resetTimer() {
        // just resets the timer when we start following the paths for the first time(or if like pause for some reason)
        timer.reset();
    }

    public int run() {
        if (actions == null || actions.size() <= 0) {
            return 0;
        }
        ArrayList<Action> step = actions.get(0);
        boolean stepDone = true;
        boolean hasHadLift = false;
        for (Action action : step) {
            if (action.timeout != null && timer.milliseconds() > action.timeout) {
                continue; // per action timeout(null is no timeout)
            }
//             if the action is not done we run it and set stepDone to false so we don't move onto the next step until all actions are completed
            if (action.perpetual) {
                action.run(hw, tm, robot, rrDrive, movement);
                if (action.getClass().getName().equals(LiftAction.class.getName())) {
                    hasHadLift = true;
                }
                continue; // if its perpetual we don't need to check if its done or have to override stepDone
            }
            if (!action.isDone(hw, tm, robot, rrDrive, movement)) {
                stepDone = false;
                if (action.getClass().getName().equals(LiftAction.class.getName())) {
                    hasHadLift = true;
                }
                action.run(hw, tm, robot, rrDrive, movement);
            }
//            stepDone = false;
//            action.run(hw, tm, robot, rrDrive, movement);
        }
        if (!hasHadLift) {
            robot.lift.setLiftPower(-Config.gravity);
        }
        tm.addData("actions", step.size());
        tm.addData("steps", actions.size());
        if (stepDone) {
            timer.reset();
            actions.remove(0);
        } // we move onto the next step

        return actions.size();
//        return false;
    }
}

abstract class Action {
    public boolean perpetual = false;
    public Double timeout = null;

    public abstract void run(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement);

    public abstract boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement);
}

class ClawAction extends Action {
    enum ClawStates {
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
                robot.claw.clawServo.setPosition(Config.clawServoBackboard);
            } else {
                robot.claw.clawServo.setPosition(Config.clawServoFloor);
            }
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
        // Claw is a bit scuffed, because of the way I setup parallel actions, since isDone is always true(as we don't know the servo positon since we aren't using axons)
        // the run function would otherwise never be called, so we call it ourselves here before returning true
        if (states == null) {
            return false; // use timeout for time for now
        }
        this.run(hw, tm, robot, rrDrive, movement);
        return true;
    }
}

class LiftAction extends Action {

    private final int height;
    private final double power;
    private final PID pid = new PID(new PID.Config(Config.liftP, Config.liftI, Config.liftD));

    public LiftAction(int height, double power) {
        this.height = height;
        this.power = power;
    }

    @Override
    public void run(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        // not gonna use liftToPos so this way we can allow for custom pid per section in the future
//        double p = Range.clip(pid.calc(height, height - robot.lift.leftLift.getCurrentPosition()), -power, power);
//        robot.lift.setLiftPower(p + Config.gravity); // gravity is F in a traditional PIDF controller
            System.out.println(robot.lift.liftToPos(height, power));
//        double power = Range.clip(pid.calc(height, robot.lift.leftLift.getCurrentPosition()) * this.power, -Math.abs(this.power), Math.abs(this.power));
//        robot.lift.leftLift.setPower(Config.liftMotorPowerDown);
//        robot.lift.rightLift.setPower(Config.liftMotorPowerMacro);
//        robot.lift.setLiftPower(Config.liftMotorPowerAuton);


//        tm.addData("liftPower", power);
//        tm.addData("liftTarget", height);
//        robot.lift.setLiftPower(-(height - robot.lift.leftLift.getCurrentPosition()) * power - Config.gravity);
//                robot.lift.leftLift.setPower(lift.power);
//                robot.lift.rightLift.setPower(lift.power);
//                robot.lift.leftLift.setTargetPosition(-lift.height);
//                robot.lift.rightLift.setTargetPosition(lift.height);
//                robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        return Math.abs(robot.lift.leftLift.getCurrentPosition() - height) < Config.liftTolerance;
    }

    @Override
    public boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        return Math.abs(height - robot.lift.leftLift.getCurrentPosition()) < Config.liftTolerance;
    }
}

// Movement Action, called MvntAction to persevere the same 10 letter length to make it more readable
class MvntAction extends Action {
    private Pose2d target = null;
    private double[] direction = null;
    private Double maxPower = null;

    public MvntAction(Pose2d target, double maxPower) {
        this.target = target;
        this.maxPower = maxPower;
    }

    public MvntAction(Pose2d target) {
        this.target = target;
        this.maxPower = Config.powerMultiplier;
    }

    public MvntAction(double x, double y, double rx) {
        this.direction = new double[]{x, y, rx};
    }

    @Override
    public void run(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        Pose2d pose = rrDrive.getPose();
        if (target != null) {
            movement.move(pose, target, new Double[]{maxPower, maxPower, maxPower}, null);
            return;
        }
        robot.drive.setDrivePowers(Drive.absoluteMovement(direction[0], direction[1], direction[2], pose.getHeading()));
    }

    @Override
    public boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        if (direction != null) {
            return false;
        }
        // same as in movement.move, just inverted to be isDone instead of continueNextLoop
        double tolerance = Config.tolerance;
        double toleranceH = Config.toleranceH;
        Pose2d pose = rrDrive.getPose();
        return !(Math.abs(target.getX() - pose.getX()) > tolerance || Math.abs(target.getY() - pose.getY()) > tolerance || Math.abs(utils.angleDifference(target.getRotation().getDegrees(), pose.getRotation().getDegrees())) > toleranceH);
    }
}
