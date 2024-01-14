package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Actor.Actor;
import org.firstinspires.ftc.teamcode.Control.Actor.ClawAction;
import org.firstinspires.ftc.teamcode.Control.Actor.MvntAction;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.ObjectDet.ObjectDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RedAutonClosePurple extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        Movement movement = new Movement(robot.drive);
        Actor actor = new Actor(hardwareMap, telemetry, robot, rrDrive, movement, 2000);

        ObjectDetector objectDetector = new ObjectDetector(hardwareMap, tel);

        double third = 1920.0 / 3;
        int dir = 1; // center by default, will get overriden

        while (!isStarted()) {
            int[] coords = objectDetector.search();
            int centerX = coords[0];
            if (centerX < third) { // left
                dir = 0;
            } else if (centerX > 2 * third) {
                dir = 2;
            } else {
                dir = 1;
            }
        }

//        if (Config.dir == 0) {
        if (dir == 0) {
            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 2000.0)
                    .add(new ClawAction(true), 750.0)
                    .add(new MvntAction(new Pose2d(0, 27.5, new Rotation2d(Math.toRadians(0)))))
                    .add(new MvntAction(new Pose2d(-8, 27, new Rotation2d(Math.toRadians(0)))))
                    .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(Math.toRadians(0)))))
                    .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(Math.toRadians(-90)))))
                    .add(new MvntAction(new Pose2d(-6, 27, new Rotation2d(Math.toRadians(-90)))))
                    .add(new ClawAction(false), 750.0)
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen), 1000.0)
                    .add(new ClawAction(true), 1000.0)
                    .add(new MvntAction(new Pose2d(36, 27, new Rotation2d(-90))))
                    .add(new ClawAction(ClawAction.ClawStates.topOpen), 1000.0);


        }

//        if (Config.dir == 1) {
        if (dir == 1) {
            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 2000.0)
                    .add(new ClawAction(true), 1000.0)
                    .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(0))))
                    .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(90))))
                    .add(new MvntAction(new Pose2d(0, 36, new Rotation2d(90))))
                    .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(90))))
                    .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(0))))
                    .add(new MvntAction(new Pose2d(0, 32.5, new Rotation2d(0))))
                    .add(new ClawAction(false), 1000.0)
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen), 1000.0)
                    .add(new ClawAction(true), 1000.0)
                    .add(new MvntAction(new Pose2d(36, 27, new Rotation2d(0))));

        }

//        if (Config.dir == 2) {
        if (dir == 2) {
            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 2000.0)
                    .add(new ClawAction(true), 750.0)
                    .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(Math.toRadians(0)))), 2500.0)
                    .add(new MvntAction(new Pose2d(18, 27, new Rotation2d(Math.toRadians(0)))), 2500.0)
                    .add(new MvntAction(new Pose2d(5.5, 27, new Rotation2d(Math.toRadians(0)))), 2500.0)
                    .add(new MvntAction(new Pose2d(5.5, 27, new Rotation2d(Math.toRadians(90)))), 2500.0)
                    .add(new ClawAction(false), 750.0)
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen), 500.0)
                    .add(new ClawAction(true), 1000.0)
                    .add(new MvntAction(new Pose2d(5, 12, new Rotation2d(90))))
                    .add(new MvntAction(new Pose2d(36, 12, new Rotation2d(90))))
                    .add(new ClawAction(ClawAction.ClawStates.topOpen), 1000.0);
        }


        waitForStart();
        actor.resetTimer();

        robot.drive.imu.resetYaw();

        while (opModeIsActive() && !isStopRequested()) {
            if (actor.run() == 0) {
                robot.drive.setDrivePowers(0, 0, 0, 0);
                rrDrive.updatePoseEstimate();
                Pose2d pose = rrDrive.getPose();
                tel.addData("finished", true);
                tel.addData("x", pose.getX());
                tel.addData("y", pose.getY());
                tel.addData("h", pose.getRotation().getDegrees());
                tel.update();
            }
            else {
                rrDrive.updatePoseEstimate();
                Pose2d pose = rrDrive.getPose();

                tel.addData("x", pose.getX());
                tel.addData("y", pose.getY());
                tel.addData("h", pose.getRotation().getDegrees());
                tel.update();
            }
        }
    }
}