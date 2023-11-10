package org.firstinspires.ftc.teamcode.Auton;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.function.Supplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class GeneralPose {
    public enum Type{MOVEMENT, LIFT, CLAW}
    private Type type;
    private Robot robot;
    private int encoderTo, clawNum, clawPower;
    public Supplier<Boolean> opModeIsActive;
    public boolean instaGo = false;//ignores if the action returns false and moves to next
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Pose2d dest;
    public GeneralPose(Robot robot, int encoderTo, Supplier<Boolean> opModeIsActive)
    {
        this.robot = robot;
        this.type = Type.LIFT;
        this.encoderTo = encoderTo;
    }
    public GeneralPose(Robot robot, int arg1, int arg2, Supplier<Boolean> opModeIsActive)
    {
        this.robot = robot;
        this.clawNum = arg1;
        this.clawPower = arg2;
        this.type = Type.CLAW;
    }
    public GeneralPose(Robot robot, HardwareMap hardwareMap, Telemetry tm, Pose2d dest, Supplier<Boolean> opModeIsActive)
    {
        this.robot = robot;
        this.hardwareMap = hardwareMap;
        this.type = Type.MOVEMENT;
        this.telemetry = tm;
        this.dest = dest;
        this.opModeIsActive = opModeIsActive;
    }
    public boolean act()
    {
        boolean ret = false;
        if(!opModeIsActive.get())
        {
            if(type == Type.CLAW)
            {
                ClawMovement cm = new ClawMovement(clawNum, clawPower);
                ret = cm.act();
            }
            else if(type == Type.LIFT)
            {
                LiftMovement lm = new LiftMovement(encoderTo);
                ret = lm.act();
            }
            else if(type == Type.MOVEMENT)
            {
                MovementAction mv = new MovementAction(dest, hardwareMap);
                ret = mv.act();
            }
            if(instaGo)
            {
                return true;
            }
        }

        return ret;
    }



    private class LiftMovement implements GeneralAction {
        int encoderTo;
        public LiftMovement(int encoderTo)
        {
            this.encoderTo = encoderTo;
        }
        public boolean act()
        {
            robot.lift.liftToPos(encoderTo, Config.liftMotorPowerMacro);
            if(Math.abs(robot.lift.rightLift.getCurrentPosition() - encoderTo) < 10)
            {
                return true;
            }
            return false;
        }
    }
    private class MovementAction implements GeneralAction{
        Movement movement;
        Pose2d dest;
        public MovementAction(Pose2d dest, HardwareMap hardwareMap)
        {
            PID.Config translationConfig = new PID.Config(Config.translationP, Config.translationI, Config.translationD);
            PID.Config rotationConfig = new PID.Config(Config.rotationP, Config.rotationI, Config.rotationD);
            SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
            this.dest = dest;
            movement = new Movement(robot.drive, rrDrive, opModeIsActive, translationConfig, rotationConfig, Config.tolerance, Config.toleranceH, telemetry);
        }
        public boolean act()
        {
            return movement.move(dest);
        }
    }
    private class ClawMovement implements GeneralAction
    {
        int clawNum,power;
        public ClawMovement(int clawNum, int power)//0 = front, 1 = back
        {
            this.clawNum = clawNum;
            this.power = power;
        }
        public boolean act()
        {
            if(clawNum == 0){
                robot.claw.topServo.setPower(this.power);
            }
            else
            {
                robot.claw.bottomServo.setPower(this.power);
            }

            return true;
        }
    }
}
interface GeneralAction
{
    public boolean act();//true = move to next, false = don't move to next
}

