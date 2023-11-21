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

import java.lang.reflect.Array;
import java.util.ArrayList;

public class Actor {
    private final HardwareMap hw;
    private final Telemetry tm;
    private final Robot robot;
    private final SampleMecanumDrive rrDrive;
    private final Movement movement;

    private final ElapsedTime timer = new ElapsedTime();

    public Actor(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        this.hw = hw;
        this.tm = tm;
        this.robot = robot;
        this.rrDrive = rrDrive;
        this.movement = movement;
    }

    private ArrayList<ArrayList<Action>> actions = new ArrayList<>();

    public void add(Action action, Double timeout, boolean parallel, boolean perpetual) {
        // TODO: add per action timeout(no need for per step timeout, as per step timeout would just be Math.max(...actionTimeouts)
        action.perpetual = perpetual; // this way both are in the add params not the action params
        action.timeout = timeout;
        if (parallel) {
            actions.get(actions.size() - 1).add(action);
        } else {
            actions.add(new ArrayList<>());
            actions.get(actions.size() - 1).add(action);
        }
    }

    public void resetTimer() {
        // just resets the timer when we start following the paths for the first time(or if like pause for some reason)
        timer.reset();
    }

    public void run() {
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

    public ClawAction(ClawStates[] states) {
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
    private final AUTON_RED_NEAR.CAFPid pid = new AUTON_RED_NEAR.CAFPid(new PID.Config(Config.liftP, Config.liftI, Config.liftD));

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
    private final Pose2d target;
    private final double maxPower;

    public MvntAction(Pose2d target, double maxPower) {
        this.target = target;
        this.maxPower = maxPower;
    }

    public MvntAction(Pose2d target) {
        this.target = target;
        this.maxPower = Config.powerMultiplier;
    }

    // TODO add an option for just driving for time(for example into the wall)
    // TODO add a correct way for setting maxPower(maybe we do it similar to how lift will have its own pid)
    @Override
    public void run(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        movement.move(target, maxPower);
    }

    @Override
    public boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        // same as in movement.move, just inverted to be isDone instead of continueNextLoop
        double tolerance = Config.tolerance;
        double toleranceH = Config.toleranceH;
        Pose2d pose = rrDrive.getPose();
        return !(Math.abs(target.getX() - pose.getX()) > tolerance || Math.abs(target.getY() - pose.getY()) > tolerance || Math.abs(utils.angleDifference(target.getRotation().getDegrees(), pose.getRotation().getDegrees())) > toleranceH);
    }
}
