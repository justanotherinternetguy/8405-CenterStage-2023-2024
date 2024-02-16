package org.firstinspires.ftc.teamcode.drive.opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp(name = "Vision Portal 3x", group = "Linear Opmode")
public class VisionPortalTriple extends LinearOpMode {

    enum State {
        GET_YAW,
        CORRECT_YAW,
        GET_POS,
        CORRECT_POS;
    }

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

        waitForStart();

        while (!isStarted() && robot.lift.lifToPosBoolean(Config.FLOOR, Config.liftMotorPowerMacro)) {}

        robot.lift.setLiftPower(Config.gravity);

        waitForStart();

        ArrayList<AprilTagDetection> lastAprilTags = new ArrayList<>();

        State state = State.GET_YAW;

        Pose2d lastPose = null;

        Double avgYaw = null;

        while (opModeIsActive() && !isStopRequested()) {
            rrDrive.update();
            Pose2d pose = rrDrive.getPose();

            ArrayList<AprilTagDetection> detections;

            switch (state) {
                case GET_YAW:
                    robot.drive.setDrivePowers(0,0,0,0);

                    detections = aprilTagProcessor.getDetections();
                    if (detections.size() == 0) continue;

                    lastAprilTags = detections;

                    avgYaw = lastAprilTags.stream()
                            .mapToDouble(a -> a.ftcPose.yaw)
                            .average().getAsDouble();

                    state = State.CORRECT_YAW;

                    lastPose = pose;

                    break;
                case CORRECT_YAW:
                    double newHeading = lastPose.getRotation().getDegrees() - avgYaw;
                    boolean finished = !movement.move(pose, new Pose2d(lastPose.getX(), lastPose.getY(), new Rotation2d(Math.toRadians(newHeading))), tel);

                    if (finished) {
                        state = State.GET_POS;
                        robot.drive.setDrivePowers(0,0,0,0);
                    }
                    break;
                case GET_POS:
                    detections = aprilTagProcessor.getDetections();
                    if (detections.size() == 0) {
                        // we can do like if we don't get any after a long time, use our current pose or lastAprilTags and trig and try to just yk, like get to where we think it should be at?
                        continue;
                    };

                    lastAprilTags = detections;
                    state = State.CORRECT_POS;
                    robot.drive.imu.resetYaw();
                    rrDrive.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(0,0,0));
                    break;
                case CORRECT_POS:

                    Double leftRel = null;
                    Double leftAbs = null;
                    Double centerRel = null;
                    Double centerAbs = null;
                    Double rightRel = null;
                    Double rightAbs = null;

                    for (AprilTagDetection tagDetection : lastAprilTags) {
                        if (tagDetection.id == 1 || tagDetection.id == 4) {
                            // left
                            leftRel = tagDetection.ftcPose.x;
                            // get the realworld pose
                        } else if (tagDetection.id == 2 || tagDetection.id == 5) {
                            // center
                        } else if (tagDetection.id == 3 || tagDetection.id == 6) {
                            // right
                        } else {
                            // shouldn't happen ever
                        }
                    }

                    break;
            }
        }
    }
}
