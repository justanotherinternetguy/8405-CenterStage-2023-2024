package org.firstinspires.ftc.teamcode.Control.Actor;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public abstract class Action {
    public boolean perpetual = false;
    public Double timeout = null;

    public abstract void run(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement);

    public abstract boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement);

    public abstract double defaultTimeout(Pose2d pose, int lift, Double prevTilt, ClawAction.ClawStates[] prevClaw);
}
