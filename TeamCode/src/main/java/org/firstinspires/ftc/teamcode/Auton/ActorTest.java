package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class ActorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        Movement movement = new Movement(robot.drive);
        Actor actor = new Actor(hardwareMap, telemetry, robot, rrDrive, movement, 5000);

        if (Config.dir == 0) {
            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 500.0)
                    .add(new ClawAction(true), 750.0)
                    .add(new MvntAction(new Pose2d(3, 27.5, new Rotation2d(Math.toRadians(-90)))))
                    .add(new MvntAction(new Pose2d(-4.5, 27, new Rotation2d(Math.toRadians(-90)))))
                    .add(new ClawAction(false), 750.0)
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen), 1000.0)
                    .add(new ClawAction(true), 750.0)
                    .add(new MvntAction(new Pose2d(12, 32.5, new Rotation2d(Math.toRadians(-90)))))
                    .add(new MvntAction(new Pose2d(24, 32.5, new Rotation2d(Math.toRadians(90)))))
                    .add(new MvntAction(new Pose2d(32, 32.5, new Rotation2d(Math.toRadians(90)))))
                    .add(new LiftAction(Config.boardBase + 100, Config.liftMotorPowerAuton))
                    .add(new MvntAction(1/3.0, 0.0, 0.0), 1000.0)
                    .add(new LiftAction(Config.boardBase + 100, Config.liftMotorPowerAuton), true, true)
                    .add(new ClawAction(ClawAction.ClawStates.topOpen), 750.0)
                    .add(new LiftAction(Config.boardBase + 100, Config.liftMotorPowerAuton), true, true)
                    .add(new MvntAction(new Pose2d(32, 31, new Rotation2d(Math.toRadians(90)))))
                    .add(new MvntAction(new Pose2d(32, 3, new Rotation2d(Math.toRadians(90)))))
                    .add(new MvntAction(new Pose2d(45, 3, new Rotation2d(Math.toRadians(90)))));
        }

        if (Config.dir == 1) {
            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 500.0)
                    .add(new ClawAction(true), 1000.0)
                    .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(0))))
                    .add(new ClawAction(false), 1000.0)
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen), 1000.0)
                    .add(new ClawAction(true), 1000.0)
                    .add(new MvntAction(new Pose2d(0, 20, new Rotation2d(0))))
                    .add(new MvntAction(new Pose2d(32, 27, new Rotation2d(Math.toRadians(90)))))
                    .add(new LiftAction(Config.boardBase + 100, Config.liftMotorPowerAuton))
                    .add(new MvntAction(0.25, 0.0, 0.0))
                    .add(new LiftAction(Config.boardBase + 100, Config.liftMotorPowerAuton), true, true)
                    .add(new ClawAction(ClawAction.ClawStates.topOpen), 1000.0)
                    .add(new LiftAction(Config.boardBase + 100, Config.liftMotorPowerAuton), true, true)
                    .add(new MvntAction(new Pose2d(32, 27, new Rotation2d(Math.toRadians(90)))))
                    .add(new MvntAction(new Pose2d(32, 3, new Rotation2d(Math.toRadians(90)))))
                    .add(new MvntAction(new Pose2d(45, 3, new Rotation2d(Math.toRadians(90)))));
        }

        if (Config.dir == 2) {
            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 500.0)
                    .add(new ClawAction(true), 750.0)
                    .add(new MvntAction(new Pose2d(4, 27, new Rotation2d(Math.toRadians(90)))), 2500.0)
                    .add(new ClawAction(false), 750.0)
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen), 500.0)
                    .add(new ClawAction(true), 750.0)
                    .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(Math.toRadians(90)))), 750.0)
                    .add(new MvntAction(new Pose2d(0, 3, new Rotation2d(Math.toRadians(90)))), 2000.0)
                    .add(new MvntAction(new Pose2d(32, 3, new Rotation2d(Math.toRadians(90)))), 2500.0)
                    .add(new MvntAction(new Pose2d(32, 21.625, new Rotation2d(Math.toRadians(90)))), 2500.0)
                    .add(new LiftAction(Config.boardBase + 100, Config.liftMotorPowerAuton))
                    .add(new MvntAction(1/3.0, 0.0, 0.0), 1750.0)
                    .add(new LiftAction(Config.boardBase + 100, Config.liftMotorPowerAuton), true, true)
                    .add(new ClawAction(ClawAction.ClawStates.topOpen), 750.0)
                    .add(new LiftAction(Config.boardBase + 100, Config.liftMotorPowerAuton), true, true)
                    .add(new MvntAction(new Pose2d(32, 21.625, new Rotation2d(Math.toRadians(90)))))
                    .add(new MvntAction(new Pose2d(32, 3, new Rotation2d(Math.toRadians(90)))), 2000.0)
                    .add(new MvntAction(new Pose2d(45, 3, new Rotation2d(Math.toRadians(90)))), 2500.0);
        }


        waitForStart();

        robot.drive.imu.resetYaw();

        while (opModeIsActive() && !isStopRequested()) {
            if (actor.run() == 0) {
                robot.drive.setDrivePowers(0,0,0,0);
                rrDrive.updatePoseEstimate();
                Pose2d pose = rrDrive.getPose();
                tel.addData("finished", true);
                tel.addData("x", pose.getX());
                tel.addData("y", pose.getY());
                tel.addData("h", pose.getRotation().getDegrees());
                tel.update();
            }
        }
    }
}