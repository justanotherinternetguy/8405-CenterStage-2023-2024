package org.firstinspires.ftc.teamcode.drive.opmodes;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;

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
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        TeamPropProcessor teamPropProcessor = new TeamPropProcessor();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//                .addProcessor(aprilTagProcessor)
//                .addProcessor(teamPropProcessor)
//                .setCameraResolution(new Size(800, 600)) <-- supported for webcam 1
                .setCameraResolution(new Size(960, 540))

                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();

        ElapsedTime timer = new ElapsedTime();

        int framecount = 0;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (timer.seconds() > framecount) {
                framecount++;
                visionPortal.saveNextFrameRaw("Frame " + framecount);
            }
//            tel.addData("latest x", teamPropProcessor.latest_x);
//            tel.addData("latest y", teamPropProcessor.latest_y);
//            tel.addData("side", teamPropProcessor.side);
            tel.addData("seconds", timer.seconds());
            tel.addData("framecount", framecount);
            tel.update();
        }
    }
}
