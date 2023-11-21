package org.firstinspires.ftc.teamcode.Auton;


import static org.firstinspires.ftc.teamcode.Auton.Config.tolerance;
import static org.firstinspires.ftc.teamcode.Auton.Config.toleranceH;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils;

public class OldActor {
    HardwareMap hw;
    Telemetry tm;
    Robot robot;
    SampleMecanumDrive rrDrive;

    ArrayList<ArrayList<OldAction>> actions = new ArrayList<>();
    ArrayList<OldAction> perpetualActions = new ArrayList<>();
    ElapsedTime timer = null;
    int indexOn = 0;
    double maxTime;
    public OldActor(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, double maxTimePerAction)
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


        OldAction x = null;
        if(a.type == ActionInput.inputType.LIFT)
        {
            x = new OldLiftAction(robot.lift, a.args[0], a.args[1]);
        }
        else if(a.type == ActionInput.inputType.CLAW)
        {
            x = new OldClawAction(robot.claw, a.args[0], a.args[1]);
        }
        else if(a.type == ActionInput.inputType.MOVEMENT)
        {
            x = new OldMovementAction(new Pose2d(new Translation2d(a.args[0], a.args[1]), new Rotation2d(a.args[2])),
                    robot,
                    rrDrive,
                    a.args[3]/100.0);
        }



        if(x != null)
        {
            ArrayList<OldAction> temp = new ArrayList<>();//creates new arraylist to update perpetualactions
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
            for(OldAction b : perpetualActions)
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
        for(OldAction a : this.actions.get(indexOn))
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
abstract class OldAction
{
    public abstract boolean act();
}

class ConfigChange extends OldAction
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
class OldLiftAction extends OldAction {

    Lift lift;
    int liftTo;
    int liftPower;
    public OldLiftAction(Lift lift, int liftTo, int liftPower)
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

class OldClawAction extends OldAction
{
    Claw claw;
    int bottom, top;
    public OldClawAction(Claw claw, int bottom, int top)
    {
        this.claw = claw;
        this.bottom = bottom;
        this.top = top;
    }

    @Override
    public boolean act() {
        double bottomPower = this.bottom==0 ? Config.bottomServoClose : Config.bottomServoOpen;
        double topPower = this.top==0 ? Config.topServoClose : Config.topServoOpen;
        claw.setPower(bottomPower, topPower);
        return true;
    }
}

class OldMovementAction extends OldAction
{
    Pose2d target;
    SampleMecanumDrive rrDrive;
    PID driveXPID, driveYPID, headingPID;
    Robot robot;
    double power;
    public OldMovementAction(Pose2d destination, Robot robot, SampleMecanumDrive rrDrive, double power)
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



