package org.firstinspires.ftc.teamcode.ObjectDet;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class ObjectDetector {
    private HardwareMap hm;
    private Telemetry telemetry;
    public OpenCvWebcam camera;
    public ObjectDetectionPipeline odp;
    public ObjectDetector(HardwareMap hm, Telemetry tm)
    {
        this.telemetry = tm;
        this.hm = hm;
        int cameraMonitorViewId = hm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hm.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hm.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        odp = new ObjectDetectionPipeline();
        camera.setPipeline(odp);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                if (Config.manualWhite) {
                    camera.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.MANUAL);
                    camera.getWhiteBalanceControl().setWhiteBalanceTemperature(Config.temp);
                } else {
                    camera.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.AUTO);
                }
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                camera.showFpsMeterOnViewport(true);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("error", errorCode);
                telemetry.update();
            }
        });
    }
    public int[] search()
    {
        return odp.getLatestCenter();
    }
    private static class ObjectDetectionPipeline extends OpenCvPipeline
    {
        int latest_x;
        int latest_y;
        public ObjectDetectionPipeline()
        {

        }
        public int[] getLatestCenter()
        {
            return new int[]{this.latest_x, this.latest_y};
        }



        @Override
        public Mat processFrame(Mat input) {Mat temp = new Mat();
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

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
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
                Imgproc.circle(input, new Point(centerX, centerY), 25, new Scalar(255, 0, 255), -1);
            }

            mask.release();
            mask2.release();
            res.release();
            finalMask.release();
            hierarchy.release();
            kernel.release();
            return input;
        }

    }
}
