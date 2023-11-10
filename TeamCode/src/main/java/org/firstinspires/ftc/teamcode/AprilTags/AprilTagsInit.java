package org.firstinspires.ftc.teamcode.AprilTags;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class AprilTagsInit {
    double fx = 369.50;
    double fy = 369.50;
    double cx = 960;
    double cy = 540;
    double tagsize = 0.0406;//meters


    private HardwareMap hm;
    private Telemetry tm;
    public OpenCvWebcam camera;
//    public OpenCvInternalCamera2 camera;
    public AprilTagsDetectionPipeline aprilTagDetect;
    public AprilTagsInit( HardwareMap hardwareMap, Telemetry tm)
    {
        this.hm = hardwareMap;
        this.tm = tm;
    }

    public void initialize(Telemetry telemetry)
    {
        int cameraMonitorViewId = hm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hm.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hm.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.FRONT, cameraMonitorViewId);
        aprilTagDetect = new AprilTagsDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetect);
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
    public void search()
    {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetect.getLatestDetections();
        if (currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections)
            {
                tm.addData("Tag Number: ", tag.id);
                tm.addData("Tag X: ", tag.center.x);
                tm.addData("Tag Y:", tag.center.y);
            }
        }
        else
        {
            tm.addLine("Don't see any tags");
        }
        tm.update();

    }
    public double[] searchFor(int id) {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetect.getLatestDetections();
        if (currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == id)
                {
                    return new double[]{tag.id, tag.center.x, tag.center.y};
                }
            }
        }
        return null;
    }


}
