package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Control.Actor.Actor;
import org.firstinspires.ftc.teamcode.Control.Actor.ClawAction;
import org.firstinspires.ftc.teamcode.Control.Actor.LiftAction;
import org.firstinspires.ftc.teamcode.Control.Actor.MvntAction;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.ObjectDet.TeamPropProcessor;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class BlueAutonClose extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        Movement movement = new Movement(robot.drive);
        Actor actor = new Actor(hardwareMap, telemetry, robot, rrDrive, movement, 3000);

        rrDrive.setPoseEstimate(new Pose2d(0, 0, 0));

        TeamPropProcessor teamPropProcessor = new TeamPropProcessor();

        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(teamPropProcessor)
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(teamPropProcessor, 60);

        while (!isStarted()) {
            tel.addData("x", teamPropProcessor.latest_x);
            tel.addData("y", teamPropProcessor.latest_y);
            tel.addData("dir", teamPropProcessor.side);
            tel.update();
        }

        if (teamPropProcessor.side == 0) { // left
            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 2500.0)
                    .add(new ClawAction(0), 1000.0)
                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5))
                    .add(new MvntAction(createPose(-18.5, 27, 90)))
//                    .add(new LiftAction(Config.FLOOR / 2, Config.liftMotorPowerAuton / 1.5))
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen))
//                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5))
                    .add(new MvntAction(createPose(-25, 27, 90)))
                    .add(new MvntAction(createPose(-36, 21, -90)))
                    .add(new LiftAction(Config.boardBase, Config.liftMotorPowerAuton))
                    .add(new ClawAction(1))
                    .add(new MvntAction(-0.25, 0.0, 0.0), 2000.0)
                    .add(new ClawAction(ClawAction.ClawStates.topOpen))
                    .add(new MvntAction(createPose(-36, 21, -90)))
                    .add(new ClawAction(0))
                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5))
                    .add(new MvntAction(createPose(-36, 3, -90)))
                    .add(new MvntAction(createPose(-44, 3, -90)));
        }
        if (teamPropProcessor.side == 1) { // center
            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 2500.0)
                    .add(new ClawAction(0), 1000.0)
                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5))
                    .add(new MvntAction(createPose(0, 31.5, 0)))
//                    .add(new LiftAction(Config.FLOOR / 2, Config.liftMotorPowerAuton / 1.5))
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen))
//                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5))
                    .add(new MvntAction(createPose(0, 24, 0)))
                    .add(new MvntAction(createPose(-36, 24, -90)))
                    .add(new LiftAction(Config.boardBase, Config.liftMotorPowerAuton), true)
                    .add(new MvntAction(createPose(-36, 28.5, -90)))
                    .add(new ClawAction(1), true)
                    .add(new MvntAction(-0.25, 0.0, 0.0), 2000.0)
                    .add(new ClawAction(ClawAction.ClawStates.topOpen))
                    .add(new MvntAction(createPose(-36, 28.5, -90)))
                    .add(new ClawAction(0), true)
                    .add(new MvntAction(createPose(-36, 3, -90)))
                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5), true)
                    .add(new MvntAction(createPose(-44, 3, -90)));
        } else if (teamPropProcessor.side == 2) { // right
            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 2500.0)
                    .add(new ClawAction(0), 1000.0)
                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5))
                    .add(new MvntAction(createPose(0, 27.5, 0)))
                    .add(new MvntAction(createPose(0, 27.5, 90)))
                    .add(new MvntAction(createPose(4, 27.5, 90)))
//                    .add(new LiftAction(Config.FLOOR / 2, Config.liftMotorPowerAuton / 1.5))
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen))
                    .add(new MvntAction(createPose(0, 27.5, 90)))
//                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5), true)
                    .add(new MvntAction(createPose(-36, 29.5, -94)))
                    .add(new LiftAction(Config.boardBase, Config.liftMotorPowerAuton), true)
                    .add(new MvntAction(-0.25, 0.0, 0.0), 2000.0)
                    .add(new ClawAction(1), true)
                    .add(new ClawAction(ClawAction.ClawStates.topOpen))
                    .add(new MvntAction(createPose(-36, 29.5, -94)))
                    .add(new MvntAction(createPose(-36, 2, -94)))
                    .add(new ClawAction(0), true)
                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5), true)
                    .add(new MvntAction(createPose(-44, 1.5, -94)));
        }

        boolean pathDone = false;

        waitForStart();

        actor.resetTimer();

        while (opModeIsActive() && !isStopRequested()) {
            rrDrive.update();
            System.out.println(actor.actions);
            if (pathDone) {
                tel.addData("State", "Done");
                tel.update();
                continue;
            }
            int pathRemaining = actor.run();
            if (pathRemaining == 0) {
                pathDone = true;
            }
            tel.addData("State", pathRemaining);
            tel.update();
        }
    }

    public com.arcrobotics.ftclib.geometry.Pose2d createPose(double x, double y, double heading) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(x, y, new Rotation2d(Math.toRadians(heading)));
    }
}
