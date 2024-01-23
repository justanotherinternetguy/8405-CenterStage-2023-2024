package org.firstinspires.ftc.teamcode.drive.opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Vision Portal Testing NEW", group = "Linear Opmode")
public class VisionPortalTeleopOther extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Movement movement = new Movement(robot.drive, new PID.Config(Config.translationP, Config.translationI, Config.translationD), new PID.Config(0.02, 0.2, 0));

        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        TeamPropProcessor teamPropProcessor = new TeamPropProcessor();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
//                .addProcessor(teamPropProcessor)
                .setCameraResolution(new Size(800, 600))
//                .setCameraResolution(new Size(960, 540))

                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();

        ElapsedTime timer = new ElapsedTime();

        int framecount = 0;

//        robot.lift.setLiftPower(Config.gravity);


        waitForStart();

        Pose2d lastAprilTagPos = new Pose2d();

        while (opModeIsActive() && !isStopRequested()) {
//            if (timer.seconds() > framecount) {
//                framecount++;
//                visionPortal.saveNextFrameRaw("Frame " + framecount);
//            }
////            tel.addData("latest x", teamPropProcessor.latest_x);
////            tel.addData("latest y", teamPropProcessor.latest_y);
////            tel.addData("side", teamPropProcessor.side);
//            tel.addData("seconds", timer.seconds());
//            tel.addData("framecount", framecount);
//            tel.update();

            rrDrive.updatePoseEstimate();
            Pose2d pose = rrDrive.getPose();

            List<AprilTagDetection> detectionList = aprilTagProcessor.getDetections();
            if (detectionList.size() == 3 && gamepad1.dpad_left) {
                AprilTagDetection det4 = detectionList.get(0);
                AprilTagPoseFtc pos4 = det4.ftcPose;
                AprilTagDetection det5 = detectionList.get(1);
                AprilTagPoseFtc pos5 = det5.ftcPose;
                AprilTagDetection det6 = detectionList.get(2);
                AprilTagPoseFtc pos6 = det6.ftcPose;

                double deltaX = pose.getX() - pos5.x;
                tel.addData("x", pos5.x);
                tel.addData("y", pos5.y);
                tel.addData("delta x", deltaX);
                double deltaY = pose.getY() - pos5.y;
                tel.addData("delta y", deltaY);

                double newX = pose.getX() + pos5.x;
                tel.addData("new X", newX);

                double newY = pose.getY() + pos5.y;
                tel.addData("new Y", newY);

                lastAprilTagPos = new Pose2d(newX, pose.getY(), new Rotation2d(pose.getHeading()));
//                lastAprilTagPos = new Pose2d(2, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
//                Drive.DrivePowers powers = Drive.absoluteMovement(newX, poseEstimate.getY(), h, -pose.getHeading());
//                drive.setDrivePowers(powers);

//                if (deltaX > 1) { //right
//                    lastAprilTagPos = new Pose2d(deltaX, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
//                }
//                if (deltaX < 1) { //right
//                    lastAprilTagPos = new Pose2d(deltaX, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
//                }
            }
            if (!movement.move(pose, lastAprilTagPos, tel)) {
                tel.addData("Done", true);
            } else {
                tel.addData("Done", false);
            }
            tel.addData("pX: ", pose.getX());
            tel.addData("pY: ", pose.getY());
            tel.addData("pH: ", pose.getRotation().getDegrees());
            tel.update();
        }
    }
}
