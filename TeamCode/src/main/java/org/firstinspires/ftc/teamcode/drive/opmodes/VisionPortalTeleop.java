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
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();

        ElapsedTime timer = new ElapsedTime();

        int framecount = 0;

//        robot.lift.setLiftPower(Config.gravity);

        robot.claw.clawServo.setPosition(Config.clawServoFloor);

        sleep(1000);

        robot.claw.topServo.setPosition(Config.topServoClose);
        robot.claw.bottomServo.setPosition(Config.bottomServoClose);

        sleep(1000);

        while (!isStarted() && Math.abs(Config.FLOOR - robot.lift.leftLift.getCurrentPosition()) > Config.liftTolerance) {
            robot.lift.liftToPos(Config.FLOOR, Config.liftMotorPowerMacro);
        }
        robot.lift.setLiftPower(Config.gravity);

//        tel.addData("ID: ", 0);
//        tel.addData("x: ", 0);
//        tel.addData("y: ", 0);
//        tel.addData("z: ", 0);
//        tel.addData("bearing: ", 0);
//        tel.addData("elevation: ", 0);
//        tel.addData("range: ", 0);
//        tel.addData("pitch: ", 0);
//        tel.addData("roll: ", 0);
//        tel.addData("yaw: ", 0);
//        tel.addData("new h:", 0);
//        tel.addData("new x:", 0);
//        tel.addData("new y:", 0);
//        tel.addData("pX: ", 0);
//        tel.addData("pY: ", 0);
//        tel.addData("pH: ", 0);
        tel.addData("tf", 0);
        tel.update();

        waitForStart();

        Pose2d lastAprilTagPos = new Pose2d();

        boolean yawHasBeenCorrected = true;
        double newYaw = Double.MAX_VALUE;
        boolean translationHasbeenCorrected = false;
        Pose2d newPose = null;
        Pose2d start = null;

        while (opModeIsActive() && !isStopRequested()) {
//            if (timer.seconds() > framecount) {
//                framecount++;
//                visionPortal.saveNextFrameRaw("Frame " + framecount);
//            }
//            tel.addData("latest x", teamPropProcessor.latest_x);
//            tel.addData("latest y", teamPropProcessor.latest_y);
//            tel.addData("side", teamPropProcessor.side);
//            tel.addData("seconds", timer.seconds());
//            tel.addData("framecount", framecount);
//            tel.update();

            rrDrive.updatePoseEstimate();
            Pose2d pose = rrDrive.getPose();

            List<AprilTagDetection> detectionList = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : detectionList) {
                if (detection.id != 5 && detection.id != 2) continue;
                AprilTagPoseFtc pos = detection.ftcPose;
//                tel.addData(String.valueOf(detection.id), "x: " + pos.x + ", y: " + pos.y + ", z:" + pos.z + ", bearing: " + pos.bearing + ", elevation:" + pos.elevation + ", pitch: " + pos.pitch + ", range: " + pos.range + ", roll: " + pos.roll + ", yaw: " + pos.yaw);
//                tel.addData("ID: ", String.valueOf(detection.id));
//                tel.addData("x: ", pos.x);
//                tel.addData("y: ", pos.y);
//                tel.addData("z: ", pos.z);

                //b=90-yaw-bearing
                //fr=sqrt(range^2-z^2)
                //sin(b)\*fr=absY
                //cos(b)\*fr=absX

                if (!yawHasBeenCorrected) {
                    if (newYaw == Double.MAX_VALUE) {
                        start = pose;
                    }
                    newYaw = pose.getRotation().getDegrees() - pos.yaw;
                } else if (!translationHasbeenCorrected & newPose == null) {
                    newPose = new Pose2d(pose.getX() + pos.x,  pose.getY() + pos.y - Config.backBoardOffset, new Rotation2d(0));
                    tel.addData("rx", pos.x);
                    tel.addData("ry", pos.y);
                    tel.addData("nx", newPose.getX());
                    tel.addData("ny", newPose.getY());
                }
            }
//            if (!movement.move(pose, lastAprilTagPos, tel)) {
//                tel.addData("Done", true);
//            } else {
//                tel.addData("Done", false);
//            }
            boolean dropped = false;

            if (!yawHasBeenCorrected & newYaw != Double.MAX_VALUE) {
                boolean tf = !movement.move(pose, new Pose2d(start.getX(), start.getY(), new Rotation2d(Math.toRadians(newYaw))), tel);
                tel.addData("state", "heading");
                tel.addData("heading", newYaw);
                tel.addData("tf", tf);
                if (!tf) {
                    robot.drive.setDrivePowers(0,0,0,0);
                    sleep(1000);
                    robot.drive.imu.resetYaw();
                    rrDrive.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(0,0,0));
                    yawHasBeenCorrected = true;
                }
            } else if (!translationHasbeenCorrected & newPose != null) {
                translationHasbeenCorrected = !movement.move(pose, newPose, telemetry);
                tel.addData("state", "translation");
            } else if (!dropped) {
                tel.addData("state", "lift1");
                tel.update();
                robot.drive.setDrivePowers(0,0,0,0);
                while (opModeIsActive() && !isStopRequested() && Math.abs(Config.liftBackBoard - robot.lift.leftLift.getCurrentPosition()) > Config.liftTolerance) {
                    robot.lift.liftToPos(Config.liftBackBoard, Config.liftMotorPowerMacro * 1.5);
                }
                robot.lift.setLiftPower(Config.gravity);
                tel.addData("state", "tilt");
                tel.update();
                robot.claw.clawServo.setPosition(Config.clawServoBackboard);
                sleep(1000);
                tel.addData("state", "move1");
                tel.update();
                robot.drive.setDrivePowers(Config.boardPower,Config.boardPower,Config.boardPower,Config.boardPower);
                sleep(3000);
                robot.drive.setDrivePowers(0,0,0,0);
                tel.addData("state", "claw1");
                tel.update();
                robot.claw.bottomServo.setPosition(Config.bottomServoOpen);
                sleep(1000);
                tel.addData("state", "moveb1");
                tel.update();
                robot.drive.setDrivePowers(-Config.boardPower*0.5,-Config.boardPower*0.5,-Config.boardPower*0.5,-Config.boardPower*0.5);
                sleep(1000);
                tel.addData("state", "lift2");
                tel.update();
                while (opModeIsActive() && !isStopRequested() && Math.abs(Config.liftBackBoard + Config.secondOffset - robot.lift.leftLift.getCurrentPosition()) > Config.liftTolerance) {
                    robot.lift.liftToPos(Config.liftBackBoard + Config.secondOffset, Config.liftMotorPowerMacro * 1.5);
                }
                robot.lift.setLiftPower(Config.gravity);
                tel.addData("state", "move2");
                tel.update();
                robot.drive.setDrivePowers(Config.boardPower,Config.boardPower,Config.boardPower,Config.boardPower);
                sleep(1000);
                tel.addData("state", "claw2");
                tel.update();
                robot.claw.topServo.setPosition(Config.topServoOpen);
                sleep(1000);

                tel.addData("state", "bailing");
                tel.update();
                robot.drive.setDrivePowers(-Config.boardPower,-Config.boardPower,-Config.boardPower,-Config.boardPower);
                sleep(1500);
                robot.drive.setDrivePowers(0,0,0,0);
                tel.addData("state", "finished");
                dropped = true;
            }
            tel.addData("pX: ", pose.getX());
            tel.addData("pY: ", pose.getY());
            tel.addData("pH: ", pose.getRotation().getDegrees());
            tel.update();
        }
    }
}
