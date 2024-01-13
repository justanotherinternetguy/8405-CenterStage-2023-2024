package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Actor.Actor;
import org.firstinspires.ftc.teamcode.Control.Actor.ClawAction;
import org.firstinspires.ftc.teamcode.Control.Actor.LiftAction;
import org.firstinspires.ftc.teamcode.Control.Actor.MvntAction;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class ActorTestBlueFar extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        Movement movement = new Movement(robot.drive);
        Actor actor = new Actor(hardwareMap, telemetry, robot, rrDrive, movement, 3000);

            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 2000.0)
                    .add(new ClawAction(true), 1000.0)
                    .add(new MvntAction(new Pose2d(0, 32.5, new Rotation2d(0))))
                    .add(new ClawAction(false), 1000.0)
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen), 1000.0)
                    .add(new ClawAction(true), 1000.0)
                    // done with purple placement
                    .add(new MvntAction(new Pose2d(0, 26, new Rotation2d(0)))) // go back
                    .add(new MvntAction(new Pose2d(20, 26, new Rotation2d(Math.toRadians(0)))))
                    .add(new MvntAction(new Pose2d(20, 53, new Rotation2d(Math.toRadians(90))))) // in front of the stack
                    .add(new LiftAction(Config.stack + 150, Config.liftMotorPowerAuton))
                    .add(new ClawAction(false), 2000.0)
                    .add(new LiftAction(Config.stack + 150, Config.liftMotorPowerAuton), true, true)
                    .add(new MvntAction(1/3.0, 0,0), 500.0)  // into wall
                    .add(new LiftAction(Config.stack + 100, Config.liftMotorPowerAuton / 2), true, true)
                    .add(new MvntAction(-0.2, 0,0), 25.0) // over stack
//                    .add(new LiftAction(Config.stack + 150, Config.liftMotorPowerAuton / 2), true, true)
                    .add(new MvntAction(0, 0,0), 100.0) // stop
                    .add(new LiftAction(Config.stack + 150, Config.liftMotorPowerAuton / 2), true, true)
                    .add(new LiftAction(Config.stack, Config.liftMotorPowerAuton / 2.0), 750.0)
                    .add(new ClawAction(ClawAction.ClawStates.bottomClosed), 2000.0)
                    .add(new LiftAction(Config.stack, Config.liftMotorPowerAuton), true, true)
                    .add(new LiftAction(Config.stack+150, Config.liftMotorPowerAuton / 1.5))
                    .add(new MvntAction(new Pose2d(20, 53, new Rotation2d(Math.toRadians(90)))))
                    .add(new LiftAction(Config.stack+150, Config.liftMotorPowerAuton / 1.5), true, true)
                    .add(new MvntAction(new Pose2d(20, 26, new Rotation2d(Math.toRadians(0)))))
                    .add(new LiftAction(Config.stack+150, Config.liftMotorPowerAuton / 1.5), true, true)
                    .add(new MvntAction(new Pose2d(0, 3, new Rotation2d(0))))
                    .add(new LiftAction(Config.stack+150, Config.liftMotorPowerAuton / 1.5), true, true);
//                    .add(new LiftAction(Config.stack + 75, Config.liftMotorPowerAuton)) // raise claw
//                    .add(new MvntAction(new Pose2d(16, 51.5, new Rotation2d(Math.toRadians(90))))) // back out
//                    .add(new LiftAction(Config.stack + 75, Config.liftMotorPowerAuton), true, true)
//                    .add(new ClawAction(true), 2000.0)
//                    .add(new LiftAction(Config.stack + 75, Config.liftMotorPowerAuton), true, true);

        waitForStart();
        actor.resetTimer();

        robot.drive.imu.resetYaw();

        while (opModeIsActive() && !isStopRequested()) {
            if (actor.run() == 0) {
                robot.drive.setDrivePowers(0,0,0,0);
                robot.lift.setLiftPower(0);
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