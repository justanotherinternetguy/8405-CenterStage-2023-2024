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

class TeamPropProcessor implements VisionProcessor {
    private double width;
    private double height;
    public double latest_x = 0;
    public double latest_y = 0;
    public int side = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Mat temp = new Mat();
        Mat mask = new Mat();
        Mat mask2 = new Mat();
        Mat res = new Mat();
        Mat finalMask = new Mat();
        Mat hierarchy = new Mat();
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);

        Scalar lowerVal = new Scalar(Config.mask1LH, Config.mask1LS, Config.mask1LV);
        Scalar upperVal = new Scalar(Config.mask1UH, Config.mask1US, Config.mask1UV);

        Scalar lowerVal2 = new Scalar(Config.mask2LH, Config.mask2LS, Config.mask2LV);
        Scalar upperVal2 = new Scalar(Config.mask2UH, Config.mask2US, Config.mask2UV);

        Core.inRange(temp, lowerVal, upperVal, mask);
        Core.inRange(temp, lowerVal2, upperVal2, mask2);
        Core.bitwise_or(mask, mask2, finalMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(6, 6));
        Imgproc.erode(finalMask, finalMask, kernel);
        Imgproc.dilate(finalMask, finalMask, kernel);

        Core.bitwise_and(input, input, res, finalMask);

        ArrayList<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(finalMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = -1;
        int maxAreaIdx = -1;

        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > maxArea) {
                maxArea = area;
                maxAreaIdx = i;
            }
        }

        if (maxAreaIdx != -1) {
            Moments mu = Imgproc.moments(contours.get(maxAreaIdx));
            int centerX = (int) (mu.get_m10() / mu.get_m00());
            int centerY = (int) (mu.get_m01() / mu.get_m00());
            this.latest_x = centerX;
            this.latest_y = centerY;
            for (int i = 0; i < 3; i ++) {
                if (latest_x < i * width / 3) side = i;
            }
            Imgproc.circle(input, new Point(centerX, centerY), 25, new Scalar(255, 0, 255), -1);
        }

        mask.release();
        mask2.release();
        res.release();
        finalMask.release();
        hierarchy.release();
        kernel.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}

@TeleOp(name = "Vision Portal", group = "Linear Opmode")
public class VisionPortalTeleop extends LinearOpMode {
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

        robot.lift.setLiftPower(Config.gravity);

        tel.addData("ID: ", 0);
        tel.addData("x: ", 0);
        tel.addData("y: ", 0);
        tel.addData("z: ", 0);
        tel.addData("bearing: ", 0);
        tel.addData("elevation: ", 0);
        tel.addData("range: ", 0);
        tel.addData("pitch: ", 0);
        tel.addData("roll: ", 0);
        tel.addData("yaw: ", 0);
        tel.addData("new heading:", 0);
        tel.addData("new x:", 0);
        tel.addData("new y:", 0);
        tel.addData("pX: ", 0);
        tel.addData("pY: ", 0);
        tel.addData("pH: ", 0);
        tel.update();

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
            for (AprilTagDetection detection : detectionList) {
                if (detection.id != 5) continue;
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

                // turn relative into absolute positions for movement
//                double newHeading = pose.getRotation().getDegrees() - pos.yaw;
//                tel.addData("new heading:", newHeading);
//                double newX = pose.getX() + pos.x;
//                tel.addData("new x:", newX);
//
//                double yoffset = 14 + 1;
//                double yError = pos.y - yoffset;
//                double newY = pose.getY() + yError;
//                tel.addData("new y:", newY);
//
////                newHeading = pose.getRotation().getDegrees();
//
////                lastAprilTagPos = new Pose2d(newX, newY, new Rotation2d(Math.toRadians(newHeading)));
//                lastAprilTagPos = new Pose2d(newX, newY, new Rotation2d(Math.toRadians(newHeading)));

                //b=90-yaw-bearing
                //fr=sqrt(range^2-z^2)
                //sin(b)\*fr=absY
                //cos(b)\*fr=absX

                double cornerAngle = 90 - Math.abs(pos.yaw + pos.bearing);
//                double cornerAngle = 90 - pos.yaw - pos.bearing; // abs x might already be negative from cosine i'll check like later today or smt
                double flatRange = Math.sqrt(Math.pow(pos.range, 2) - Math.pow(pos.z, 2));
                double absoluteX = Math.cos(Math.toRadians(cornerAngle)) * flatRange;
                double absoluteY = Math.sin(Math.toRadians(cornerAngle)) * flatRange;

                // calculate if x is positive or negative
                // if(90+bearing+yaw > 90) + else -
                if (cornerAngle < 90) absoluteX = absoluteX * -1;

                double newX = pose.getX() + absoluteX;
                double newY = pose.getY() + absoluteY;
                double newHeading = pose.getRotation().getDegrees() - pos.yaw;

                lastAprilTagPos = new Pose2d(newX, newY, new Rotation2d(Math.toRadians(newHeading)));
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
