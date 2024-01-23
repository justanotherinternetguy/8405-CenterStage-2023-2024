package org.firstinspires.ftc.teamcode.drive.opmodes;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "VP: 3 Apriltag Alignment Test", group = "Linear Opmode")
public class VPThreeApriltagAlignment extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Pose2d[] path = null;
        Telemetry dashTel = FtcDashboard.getInstance().getTelemetry();
        Movement movement = new Movement(robot.drive, new PID.Config(Config.translationP, Config.translationI, Config.translationD), new PID.Config(0.02, 0.2, 0));

        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();


        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();

        ElapsedTime timer = new ElapsedTime();

        int framecount = 0;

//        robot.lift.setLiftPower(Config.gravity + 0.15);


        waitForStart();

        Pose2d lastAprilTagPos = new Pose2d();

        while (opModeIsActive() && !isStopRequested()) {
            rrDrive.updatePoseEstimate();
            Pose2d pose = rrDrive.getPose();

            List<AprilTagDetection> detectionList = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : detectionList) {
                if (detection.id == 5) {
                    AprilTagPoseFtc pos = detection.ftcPose;
//                tel.addData(String.valueOf(detection.id), "x: " + pos.x + ", y: " + pos.y + ", z:" + pos.z + ", bearing: " + pos.bearing + ", elevation:" + pos.elevation + ", pitch: " + pos.pitch + ", range: " + pos.range + ", roll: " + pos.roll + ", yaw: " + pos.yaw);
                    tel.addData("ID: ", String.valueOf(detection.id));
                    tel.addData("x: ", pos.x);
                    tel.addData("y: ", pos.y);
                    tel.addData("z: ", pos.z);
                    tel.addData("bearing: ", pos.bearing);
                    tel.addData("elevation: ", pos.elevation);
                    tel.addData("range: ", pos.range);
                    tel.addData("pitch: ", pos.pitch);
                    tel.addData("roll: ", pos.roll);
                    tel.addData("yaw: ", pos.yaw);
                    tel.addData("det center x", detection.center.x);
                    tel.addData("det center y", detection.center.y);

                    if (path == null && detection.center.x > 370 && detection.center.x < 420) {
                        if (gamepad1.x) {
                            lastAprilTagPos = new Pose2d(12, 0, new Rotation2d(Math.toRadians(0)));
                        }
                    }
                }
            }


//            AprilTagDetection det5 = detectionList.get(1);
//            AprilTagPoseFtc pos5 = det5.ftcPose;
//            tel.addData("ID: ", String.valueOf(det5.id));
//            tel.addData("x: ", pos5.x);
//            tel.addData("y: ", pos5.y);
//            tel.addData("z: ", pos5.z);
//            tel.addData("bearing: ", pos5.bearing);
//            tel.addData("elevation: ", pos5.elevation);
//            tel.addData("range: ", pos5.range);
//            tel.addData("pitch: ", pos5.pitch);
//            tel.addData("roll: ", pos5.roll);
//            tel.addData("yaw: ", pos5.yaw);


//                lastAprilTagPos = new Pose2d(newX, newY, new Rotation2d(Math.toRadians(newHeading)));
//            if (!movement.move(pose, lastAprilTagPos, tel)) {
//                tel.addData("Done", true);
//            } else {
//                tel.addData("Done", false);
//            }
            tel.addData("pX: ", pose.getX());
            tel.addData("pY: ", pose.getY());
            tel.addData("pH: ", pose.getRotation().getDegrees());
            telemetry.addData("pose", rrDrive.getPose().toString());
            telemetry.update();
            rrDrive.updatePoseEstimate();
            if (!movement.move(pose, lastAprilTagPos, tel)) {
                tel.addData("Done", true);
            } else {
                tel.addData("Done", false);
            }
            tel.update();
        }
    }
}
