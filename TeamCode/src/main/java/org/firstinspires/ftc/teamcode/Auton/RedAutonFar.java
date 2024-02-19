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
public class RedAutonFar extends LinearOpMode {
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
                    .add(new MvntAction(createPose(0, 27.5, 0)))
                    .add(new MvntAction(createPose(0, 27.5, -90)))
                    .add(new MvntAction(createPose(-5, 27.5, -90)))
//                    .add(new LiftAction(Config.FLOOR / 2, Config.liftMotorPowerAuton / 1.5))
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen))
                    .add(new LiftAction(Config.boardBase / 2, Config.liftMotorPowerAuton / 1.5))
                    .add(new MvntAction(createPose(-2, 27.5, -90)))
//                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5), true)
                    .add(new MvntAction(createPose(-3, 51, -90)))
                    .add(new MvntAction(createPose(72, 51, -90)))
                    .add(new MvntAction(createPose(74, 33, 90)))
                    .add(new LiftAction(Config.boardBase + 35, Config.liftMotorPowerMacro), true)
                    .add(new MvntAction(0.25, 0.0, 0.0), 2000.0)
                    .add(new ClawAction(1), true)
                    .add(new ClawAction(ClawAction.ClawStates.topOpen))
                    .add(new MvntAction(createPose(74, 33, 90)))
                    .add(new MvntAction(createPose(74, 3, 90)))
                    .add(new ClawAction(0), true)
                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5), true)
                    .add(new MvntAction(createPose(86, 2, 90)));
        }
        if (teamPropProcessor.side == 1) { // center
            actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 2500.0)
                    .add(new ClawAction(0), 1000.0)
                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5))
                    .add(new MvntAction(createPose(0, 46, 180)))
//                    .add(new LiftAction(Config.FLOOR / 2, Config.liftMotorPowerAuton / 1.5))
                    .add(new ClawAction(ClawAction.ClawStates.bottomOpen))
//                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5), true)
                    .add(new MvntAction(createPose(0, 54, 180)))
                    .add(new MvntAction(createPose(0, 51, -90)))
                    .add(new MvntAction(createPose(72, 51, -90)))
                    .add(new MvntAction(createPose(74, 28.5, 90)))
                    .add(new LiftAction(Config.boardBase, Config.liftMotorPowerAuton), true)
                    .add(new MvntAction(0.25, 0.0, 0.0), 2000.0)
                    .add(new ClawAction(1), true)
                    .add(new ClawAction(ClawAction.ClawStates.topOpen))
                    .add(new MvntAction(createPose(74, 28.5, 90)))
                    .add(new MvntAction(createPose(74, 3, 90)))
                    .add(new ClawAction(0), true)
                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5), true)
                    .add(new MvntAction(createPose(86, 2, 90)));;
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
                    .add(new MvntAction(createPose(0, 51, -90)))
                    .add(new MvntAction(createPose(72, 51, -90)))
                    .add(new MvntAction(createPose(84, 21, 90)))
                    .add(new LiftAction(Config.boardBase, Config.liftMotorPowerAuton), true)
                    .add(new MvntAction(0.25, 0.0, 0.0), 2000.0)
                    .add(new ClawAction(1), true)
                    .add(new ClawAction(ClawAction.ClawStates.topOpen))
                    .add(new MvntAction(createPose(84, 21, 90)))
                    .add(new MvntAction(createPose(84, 3, 90)))
                    .add(new ClawAction(0), true)
                    .add(new LiftAction(Config.FLOOR, Config.liftMotorPowerAuton / 1.5), true)
                    .add(new MvntAction(createPose(96, 2, 90)));
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
