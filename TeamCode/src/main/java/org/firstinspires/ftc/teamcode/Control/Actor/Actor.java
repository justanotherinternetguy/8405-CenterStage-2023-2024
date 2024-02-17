package org.firstinspires.ftc.teamcode.Control.Actor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class Actor {
    private final HardwareMap hw;
    private final Telemetry tm;
    private final Robot robot;
    private final SampleMecanumDrive rrDrive;
    private final Movement movement;

    private final ElapsedTime timer = new ElapsedTime();

    private final double defaultTimeout;

    private double prevTilt = 0;

    private final ClawAction.ClawStates[] prevClaw = new ClawAction.ClawStates[2];

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
//        action.timeout = timeout != null ? timeout : defaultTimeout;
        if (timeout != null) {
            action.timeout = timeout;
        } // otherwise action.timeout will be Double.NEGATIVE_INFINITY and set at runtime to globalDefault or actionDefault
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
        if (actions == null || actions.size() == 0) {
            return 0;
        }
        ArrayList<Action> step = actions.get(0); // [mnvt]
        boolean stepDone = true; // stepDone = true
        boolean hasHadLift = false; // hasHadLift = false
        for (Action action : step) {
            // action = mvnt

            // we can remove this check every time we run later
            if (action.timeout == Double.NEGATIVE_INFINITY) {
                // assign timeout at runtime
                double actionDefault = action.defaultTimeout(this.rrDrive.getPose(), robot.lift.leftLift.getCurrentPosition(), this.prevTilt, this.prevClaw);
                // actionDefault = 3000;
                if (actionDefault != -1) {
                    action.timeout = actionDefault; // action.timeout = 3000
                }
            }
            if (action instanceof ClawAction) {
                if (((ClawAction) action).states != null) {
                    for (ClawAction.ClawStates state : ((ClawAction) action).states) {
                        if (state == ClawAction.ClawStates.topOpen || state == ClawAction.ClawStates.topClosed) {
                            this.prevClaw[0] = state;
                        } else {
                            this.prevClaw[1] = state;
                        }
                    }
                } else {
                    this.prevTilt = ((ClawAction) action).tilt;
                }
            }

            if (timer.milliseconds() > action.timeout || (action.timeout == Double.NEGATIVE_INFINITY && timer.milliseconds() > this.defaultTimeout)) {
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
            //
            if (!action.isDone(hw, tm, robot, rrDrive, movement)) {
                stepDone = false;
                if (action.getClass().getName().equals(LiftAction.class.getName())) {
                    hasHadLift = true;
                }
                action.run(hw, tm, robot, rrDrive, movement);
            }
        }
        if (!hasHadLift) {
            robot.lift.setLiftPower(Config.gravity);
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

