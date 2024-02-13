package org.firstinspires.ftc.teamcode.drive.opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

import java.util.ArrayList;

@TeleOp(name = "AT Trig", group = "Linear Opmode")
public class AprilTagTrigonometry extends LinearOpMode {
    Robot robot = new Robot(hardwareMap, gamepad1);
    SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
    Movement movement = new Movement(robot.drive, new PID.Config(Config.translationP, Config.translationI, Config.translationD), new PID.Config(0.02, 0.2, 0));
    AprilTagLibrary tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();

    @Override
    public void runOpMode() {
        // AprilTagProcessor is actually just an abstract class, AprilTagProcessorImpl is the real behind the scenes class
        AprilTagProcessorImpl aprilTagProcessor = (AprilTagProcessorImpl) new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(tagLibrary) // exact same as default, just make sure if we pull from upstream stuff won't break
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

        rrDrive.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(0, 0, 0));

        while (!isStarted() && robot.lift.lifToPosBoolean(Config.FLOOR, Config.liftMotorPowerMacro)) {
        }

        robot.lift.setLiftPower(Config.gravity);

        Pose2d target = new Pose2d();

        Boolean redBoard = null;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // for now its just the 3 tag columns and not the full 6/7
            ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            if (detections.size() > 0) {
                ArrayList<com.acmerobotics.roadrunner.geometry.Pose2d> poses = new ArrayList<>();

                for (AprilTagDetection detection : detections) {
                    if (detection.id == 1 || detection.id == 2 || detection.id == 3) {
                        if (redBoard == null) {
                            redBoard = false;
                        }
                        poses.add(getPoseEstimate(detection));
                    } else if (detection.id == 4 || detection.id == 5 || detection.id == 6) {
                        if (redBoard == null) {
                            redBoard = true;
                        }
                        poses.add(getPoseEstimate(detection));
                    }
                    // otherwise its some other tag like say the stack ones
                }

                com.acmerobotics.roadrunner.geometry.Pose2d avgPose = poses.stream().reduce(new com.acmerobotics.roadrunner.geometry.Pose2d(), com.acmerobotics.roadrunner.geometry.Pose2d::plus).div(poses.size());

//                rrDrive.setPoseEstimate(avgPose);
                // return new com.arcrobotics.ftclib.geometry.Pose2d(-pose.getY(), pose.getX(), new Rotation2d(-pose.getHeading()));
                // 5, -4, -90
                // 4, 5, 90
                // y, -x, -h
                rrDrive.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(avgPose.getY(), -avgPose.getX(), -avgPose.getHeading()));
            } else {
                rrDrive.updatePoseEstimate();
            }

            if (redBoard != null) {
                VectorF columnVector = null;
                switch (Config.boardColumn) {
                    case 0:
                        if (redBoard) {
                            columnVector = tagLibrary.lookupTag(4).fieldPosition; // RedAllianceLeft
                        } else {
                            columnVector = tagLibrary.lookupTag(1).fieldPosition; // BlueAllianceLeft
                        }
                        break;
                    case 1:
                        if (redBoard) {
                            columnVector = tagLibrary.lookupTag(5).fieldPosition; // RedAllianceCenter
                        } else {
                            columnVector = tagLibrary.lookupTag(2).fieldPosition; // BlueAllianceCenter
                        }
                        break;
                    case 2:
                        if (redBoard) {
                            columnVector = tagLibrary.lookupTag(6).fieldPosition; // RedAllianceRight
                        } else {
                            columnVector = tagLibrary.lookupTag(3).fieldPosition; // BlueAllianceRight
                        }
                        break;
                    default:
                        RobotLog.setGlobalErrorMsg("Config.boardColumn must be 0-2");
                        stop();
                }

                assert columnVector != null;
                Pose2d columnPose = new Pose2d(columnVector.get(0), columnVector.get(1), new Rotation2d(0));

                target = new Pose2d(columnPose.getX(), columnPose.getY() - Config.backBoardOffset, new Rotation2d(0));

                Pose2d pose = rrDrive.getPose();
                movement.move(pose, target, tel);
            } else {
                robot.drive.setDrivePowers(0, 0, 0, 0);
            }
        }
    }

    public static com.acmerobotics.roadrunner.geometry.Pose2d getPoseEstimate(AprilTagDetection tagDetection) {
        // see https://www.desmos.com/calculator/snmimlzzvo
        // might be off as the desmos manually calculates most things which maybe different from the detection's ftcPose calculation methods
        AprilTagPoseFtc ftcPose = tagDetection.ftcPose;
        double yaw = ftcPose.yaw;
        double bearing = ftcPose.bearing;
        double range = ftcPose.range;

        double a = -yaw;
        double b = bearing;
        double c = a + b;

        double x2 = -range * Math.cos(Math.toRadians(c));
        double y2 = -range * Math.sin(Math.toRadians(c));

        VectorF tagPos = tagDetection.metadata.fieldPosition;
        double tagX = tagPos.get(0);
        double tagY = tagPos.get(1);

        double newX = tagX + x2;
        double newY = tagY + y2;

        return new com.acmerobotics.roadrunner.geometry.Pose2d(newX, newY, ftcPose.yaw);
    }
}
