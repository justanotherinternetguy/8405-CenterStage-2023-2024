package org.firstinspires.ftc.teamcode.Control.Actor;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Movement Action, called MvntAction to persevere the same 10 letter length to make it more readable
public class MvntAction extends Action {
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
            rrDrive.update();
            Pose2d pose = rrDrive.getPose();
            robot.drive.setDrivePowers(Drive.absoluteMovement(direction[0], direction[1], direction[2], -pose.getHeading()));
        }
    }

    @Override
    public boolean isDone(HardwareMap hw, Telemetry tm, Robot robot, SampleMecanumDrive rrDrive, Movement movement) {
        if (direction != null) {
            return false;
        }
        // same as in movement.move, just inverted to be isDone instead of continueNextLoop
        rrDrive.update();
        Pose2d pose = rrDrive.getPose();
        return !movement.move(pose, target, new Double[]{maxPower, maxPower, maxPower}, tm);
    }

    @Override
    public double defaultTimeout(Pose2d pose, int lift, Double prevTilt, ClawAction.ClawStates[] prevClaw) {
//        double headingDiff = Math.abs(PID.rotationGetError(pose.getRotation().getDegrees(), target.getRotation().getDegrees()));
//        double xDiff = pose.getX() - target.getX();
//        double yDiff = pose.getY() - target.getY();
//        double euclideanDistance = Math.hypot(xDiff, yDiff);
//        double translationTime = euclideanDistance * 125;
//        double headingTime = headingDiff * 20;
//        return translationTime + headingTime;
        return 3000;
    }
}
