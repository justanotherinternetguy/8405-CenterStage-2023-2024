package org.firstinspires.ftc.teamcode.Auton;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
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
        return this.add(action, parallel);
    }

    public Actor add(Action action, double timeout, boolean parallel) {
        return this.add(action, timeout, parallel, false);
    }

    public Actor add(Action action, double timeout) {
        return this.add(action, timeout, false, false);
    }

    public Actor add(Action action) {
        return this.add(action);
    }

    public void resetTimer() {
        // just resets the timer when we start following the paths for the first time(or if like pause for some reason)
        timer.reset();
    }

    public boolean run() {
        ArrayList<Action> step = actions.get(0);
        boolean stepDone = true;
        for (Action action : step) {
            if (action.timeout != null && timer.seconds() > action.timeout) {
                continue; // per action timeout(null is no timeout)
            }
            // if the action is not done we run it and set stepDone to false so we don't move onto the next step until all actions are completed
            if (action.perpetual) {
                action.run(hw, tm, robot, rrDrive, movement);
                continue; // if its perpetual we don't need to check if its done or have to override stepDone
            }
            if (!action.isDone(hw, tm, robot, rrDrive, movement)) {
                stepDone = false;
                action.run(hw, tm, robot, rrDrive, movement);
            }
        }
        if (stepDone) {
            timer.reset();
            actions.remove(0);
        } // we move onto the next step

        return actions.size() == 0;
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

    private final ClawStates[] states;

    public ClawAction(ClawStates... states) {
        this.states = states;
    }

    @Override
    public void run(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        for (ClawStates state : states) {
            switch (state) {
                case topOpen -> robot.claw.topServo.setPower(Config.topServoOpen);
                case topClosed -> robot.claw.topServo.setPower(Config.topServoClose);
                case bottomOpen -> robot.claw.bottomServo.setPower(Config.bottomServoOpen);
                case bottomClosed -> robot.claw.bottomServo.setPower(Config.bottomServoClose);
            }
        }
    }

    @Override
    public boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        // Claw is a bit scuffed, because of the way I setup parallel actions, since isDone is always true(as we don't know the servo positon since we aren't using axons)
        // the run function would otherwise never be called, so we call it ourselves here before returning true
        this.run(hw, tm, robot, rrDrive, movement);
        return true;
    }
}

class LiftAction extends Action {

    private final int height;
    private final double power;
    private final AUTON_BLUE_NEAR.CAFPid pid = new AUTON_BLUE_NEAR.CAFPid(new PID.Config(Config.liftP, Config.liftI, Config.liftD));

    public LiftAction(int height, double power) {
        this.height = height;
        this.power = power;
    }

    @Override
    public void run(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        // not gonna use liftToPos so this way we can allow for custom pid per section in the future
        double p = Range.clip(pid.calc(height, height - robot.lift.leftLift.getCurrentPosition()), -power, power);
        robot.lift.setLiftPower(p + Config.gravity); // gravity is F in a traditional PIDF controller
    }

    @Override
    public boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        return height - robot.lift.leftLift.getCurrentPosition() < Config.liftTolerance;
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
        if (target == null) {
            movement.move(target, maxPower);
        }
        Pose2d pose = rrDrive.getPose();
        double[] powers = Movement.absMovement(direction[0], direction[1], direction[2], pose.getHeading());
        robot.drive.setDrivePowers(powers[0], powers[1], powers[2], powers[3]);
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
