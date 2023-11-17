package org.firstinspires.ftc.teamcode.Auton;


import static org.firstinspires.ftc.teamcode.Auton.Config.tolerance;
import static org.firstinspires.ftc.teamcode.Auton.Config.toleranceH;

import android.widget.ArrayAdapter;

import com.acmerobotics.dashboard.FtcDashboard;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils;

public class Actor {
    HardwareMap hw;
    Telemetry tm;
    Robot robot;
    SampleMecanumDrive rrDrive;

    ArrayList<ArrayList<Action>> actions = new ArrayList<>();
    ArrayList<Action> perpetualActions = new ArrayList<>();
    ElapsedTime timer = null;
    int indexOn = 0;
    double maxTime;
    public Actor(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, double maxTimePerAction)
    {
        this.hw = hw;
        this.tm = tm;
        this.robot = robot;
        this.rrDrive = rrDrive;
        this.maxTime = maxTimePerAction;
    }
    public void add(ActionInput a)
    {
        this.add(a, false, false);
    }
    public void add(ActionInput a, boolean inParallel)
    {
        this.add(a, inParallel, false);
    }
    public void add(ActionInput a, boolean inParallel, boolean perpetual)//inParallel = in parallel with the last listed action. don't use inparallel if its the first action
    {
        if(!inParallel) // if separate steps
        {
            this.actions.add(new ArrayList<>()); // add completely new step to actions
        }


        Action x = null;
        if(a.type == ActionInput.inputType.LIFT)
        {
            x = new LiftAction(robot.lift, a.args[0], a.args[1]);
        }
        else if(a.type == ActionInput.inputType.CLAW)
        {
            x = new ClawAction(robot.claw, a.args[0]);
        }
        else if(a.type == ActionInput.inputType.MOVEMENT)
        {
            x = new MovementAction(new Pose2d(new Translation2d(a.args[0], a.args[1]), new Rotation2d(a.args[2])),
                    robot,
                    rrDrive,
                    a.args[3]/100.0);
        }



        if(x != null)
        {
            ArrayList<Action> temp = new ArrayList<>();//creates new arraylist to update perpetualactions
            for(int i = 0; i<this.perpetualActions.size(); i++)
            {
                if(!perpetualActions.get(i).getClass().getName().equals(x.getClass().getName())) {
                    temp.add(perpetualActions.get(i));//add already perpetual actions and does overriding
                }
            }
            if(perpetual)
            {
                temp.add(x);//will add this as a new perpetual action if necessary
            }
            this.perpetualActions = temp;
        }

        if(!inParallel)
        {
            for(Action b : perpetualActions)
            {
                if(b != x)
                {
                    this.actions.get(actions.size()-1).add(b);
                }
            }
        }
        this.actions.get(actions.size()-1).add(x);
    }
    public void clearPerpetual()
    {
        this.perpetualActions = new ArrayList<>();
    }
    public void add(ConfigChange c, boolean inParallel)
    {
        if(!inParallel)
        {
            this.actions.add(new ArrayList<>());
        }
        this.actions.get(actions.size()-1).add(c);
    }
    public double actingFor()
    {
        if(timer!=null)
        {
            return timer.milliseconds();
        }
        return -1;
    }



    public void act()
    {
        boolean finished = true;
        if(indexOn == actions.size())
        {
            return;
        }
        for(Action a : this.actions.get(indexOn))
        {
            finished = finished && a.act(); // before was &
            if(timer == null)
            {
                timer = new ElapsedTime();
            }
        }
        if(finished || timer.milliseconds() > maxTime)
        {
            indexOn++;
            timer = new ElapsedTime();
        }
    }

    static class ActionInput
    {
        public enum inputType{LIFT, CLAW, MOVEMENT};
        inputType type;
        int[] args;
        public ActionInput(inputType type, int[] args)
        {
            this.type=type;
            this.args=args;
        }
    }
}
abstract class Action
{
    public abstract boolean act();
}

class ConfigChange extends Action
{
    Runnable run;
    public ConfigChange(Runnable run)
    {
        this.run = run;
    }
    public boolean act()
    {
        run.run();
        return true;
    }
}
class LiftAction extends Action{

    Lift lift;
    int liftTo;
    int liftPower;
    public LiftAction(Lift lift, int liftTo, int liftPower)
    {
        this.lift = lift;
        this.liftTo = liftTo;
        this.liftPower = liftPower;

    }
    @Override
    public boolean act() {
        lift.liftToPos(liftTo, liftPower);
        if(Math.abs(lift.rightLift.getCurrentPosition() - liftTo) < 15)
        {
            return true;
        }
        return false;
    }
}

class ClawAction extends Action
{
    Claw claw;
    int arg;
    public ClawAction(Claw claw, int arg)
    {
        this.claw = claw;
        this.arg = arg;
    }

    @Override
    public boolean act() {
        if(arg == 0)
        {
            this.claw.setPower(Config.bottomServoClose, Config.topServoClose);
        }
        else if(arg == 1){
            this.claw.setPower(Config.bottomServoOpen, Config.topServoOpen);
        }
        return true;
    }
}

class MovementAction extends Action
{
    Pose2d target;
    SampleMecanumDrive rrDrive;
    PID driveXPID, driveYPID, headingPID;
    Robot robot;
    double power;
    public MovementAction(Pose2d destination, Robot robot, SampleMecanumDrive rrDrive, double power)
    {
        this.target = destination;
        this.robot=robot;
        this.rrDrive=rrDrive;
        this.driveXPID = new PID(Config.translationP,Config.translationI,Config.translationD);
        this.driveYPID = new PID(Config.translationP,Config.translationI,Config.translationD);
        this.headingPID = new PID(Config.rotationP,Config.rotationI,Config.rotationD);
        this.power=power;

    }
    @Override
    public boolean act() {
        rrDrive.update();
        Pose2d pose = rrDrive.getPose();

        double x = driveXPID.getValue(target.getX() - pose.getX());
        double y = driveYPID.getValue(target.getY() - pose.getY());
        double rx = headingPID.getValue(utils.angleDifference(target.getRotation().getDegrees(), Math.toDegrees(pose.getHeading())));
        double botHeading = -pose.getRotation().getRadians();    //i won't change it cause it seems to work but y?
        // just smt that happened when trying to get two wheel working, fixed in getPose()

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * Config.XMULTI;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        //dont spam telemetry on the actual driver hub, use dash if u want all this data

        robot.drive.setDrivePowers(frontLeftPower*power, frontRightPower*power, backLeftPower*power, backRightPower*power);
        //returns if we're there for the outside loop. can easily change to &&'s(which I recommend)

        return Math.abs(target.getX() - pose.getX()) > tolerance || Math.abs(target.getY() - pose.getY()) > tolerance || Math.abs(utils.angleDifference(target.getRotation().getDegrees(), pose.getRotation().getDegrees())) > toleranceH;
    }


}



