package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
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
public class REDAUTONNEAR extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        Movement movement = new Movement(robot.drive);
        Actor actor = new Actor(hardwareMap, telemetry, robot, rrDrive, movement, 3000);
        rrDrive.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(0, 0, 0));
        TeamPropProcessor teamPropProcessor = new TeamPropProcessor();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(teamPropProcessor)
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();



        waitForStart();
        actor.resetTimer();

        robot.drive.imu.resetYaw();

        while (opModeIsActive() && !isStopRequested()) {
            tel.addData("x", teamPropProcessor.latest_x);
            tel.addData("y", teamPropProcessor.latest_y);
            tel.addData("dir", teamPropProcessor.dir);
            if (teamPropProcessor.dir == 2) {
                actor.add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed), 1000.0)
                        .add(new LiftAction(Config.FLOOR, Config.gravity))
                        .add(new MvntAction(new Pose2d(0, 32, new Rotation2d(0))))
                        .add(new ClawAction(ClawAction.ClawStates.bottomOpen, ClawAction.ClawStates.topClosed), 1000.0)
                        .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(0))))
                        .add(new MvntAction(new Pose2d(0, 27, new Rotation2d(90))));
            }
            rrDrive.update();
            if (actor.run() == 0) {
                rrDrive.update();
                robot.drive.setDrivePowers(0, 0, 0, 0);
                Pose2d pose = rrDrive.getPose();
                tel.addData("finished", true);
                tel.addData("x", pose.getX());
                tel.addData("y", pose.getY());
                tel.addData("h", pose.getRotation().getDegrees());
                tel.update();
            }
            tel.update();
        }
    }
}
