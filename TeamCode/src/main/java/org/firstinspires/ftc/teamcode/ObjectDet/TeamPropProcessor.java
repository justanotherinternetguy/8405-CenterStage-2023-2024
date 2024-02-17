package org.firstinspires.ftc.teamcode.ObjectDet;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class TeamPropProcessor implements VisionProcessor, CameraStreamSource {
    private double width;
    private double height;
    public double latest_x = 0;
    public double latest_y = 0;
    public int side = -1;

    // FTCDashboard Stream
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
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
            for (int i = 0; i < 3; i++) {
                if (latest_x < (i + 1) * width / 3) {
                    side = i;
                    break;
                }
                System.out.println(latest_x + " | " + i + " | " + width / 3 + " : " + i * width / 3);
            }
            Imgproc.circle(input, new Point(centerX, centerY), 25, new Scalar(255, 0, 255), -1);
        }

        // FTCDashboard camera stream
        Bitmap b = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, b);
        lastFrame.set(b);

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

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
