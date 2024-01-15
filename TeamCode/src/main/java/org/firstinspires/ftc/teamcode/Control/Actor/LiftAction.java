package org.firstinspires.ftc.teamcode.Control.Actor;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class LiftAction extends Action {

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
        System.out.println(robot.lift.liftToPos(height, power));


    }

    @Override
    public boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        return Math.abs(height - robot.lift.leftLift.getCurrentPosition()) < Config.liftTolerance;
    }

    @Override
    public double defaultTimeout(Pose2d pose, int lift, Double prevTilt, ClawAction.ClawStates[] prevClaw) {
        int distance = this.height - lift;
        double distanceTimed = distance * this.power;
        return distanceTimed * 0.75;
    }
}
