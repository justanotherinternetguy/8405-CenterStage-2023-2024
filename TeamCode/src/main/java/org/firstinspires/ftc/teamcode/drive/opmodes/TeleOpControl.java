package org.firstinspires.ftc.teamcode.drive.opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Mecanum Drive", group = "Linear Opmode")
public class TeleOpControl extends LinearOpMode {
    public boolean slowMode = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime timer = new ElapsedTime();
        Movement movement = new Movement(robot.drive, new PID.Config(Config.translationP, Config.translationI, Config.translationD + 0.3), new PID.Config(0.02, 0.2, 0));

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();

        boolean lastX = false;

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


        waitForStart();
        robot.drive.imu.resetYaw();
        timer.reset();
        Pose2d lastAprilTagPos = null;
        Pose2d[] path = null;

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d poseEstimate = drive.getPose();

            List<AprilTagDetection> detectionList = aprilTagProcessor.getDetections();


            if (detectionList.size() == 3 && gamepad1.dpad_left) {
                AprilTagDetection det4 = detectionList.get(0);
                AprilTagPoseFtc pos4 = det4.ftcPose;
                AprilTagDetection det5 = detectionList.get(1);
                AprilTagPoseFtc pos5 = det5.ftcPose;
                AprilTagDetection det6 = detectionList.get(2);
                AprilTagPoseFtc pos6 = det6.ftcPose;

                double deltaX = poseEstimate.getX() - pos5.x;
                tel.addData("x", pos5.x);
                tel.addData("delta x", deltaX);
                double deltaY = poseEstimate.getY() - pos5.y;
                tel.addData("delta y", deltaY);

                double newX = poseEstimate.getX() + pos5.x;
                tel.addData("new X", newX);

//                lastAprilTagPos = new Pose2d(newX, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
//                lastAprilTagPos = new Pose2d(2, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
                path = new Pose2d[]{
                        new Pose2d(2, 0, new Rotation2d(0))
                };
//                Drive.DrivePowers powers = Drive.absoluteMovement(newX, poseEstimate.getY(), h, -pose.getHeading());
//                drive.setDrivePowers(powers);

//                if (deltaX > 1) { //right
//                    lastAprilTagPos = new Pose2d(deltaX, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
//                }
//                if (deltaX < 1) { //right
//                    lastAprilTagPos = new Pose2d(deltaX, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
//                }
            }

            robot.lift.liftTeleOp(gamepad1, tel);

            robot.claw.input(gamepad1, tel);
            robot.hang.input(gamepad1, this::opModeIsActive, this::opModeIsActive, timer);
            robot.drone.input(gamepad1, timer);

            if (gamepad1.x && !lastX) {
                slowMode = !slowMode;
            }
            lastX = gamepad1.x;

            if (!Config.fieldCentric) {
                double power = -gamepad1.left_stick_y; // remember this is reversed
                double strafe = gamepad1.left_stick_x * 1.5; // counteract imperfect strafing
                double turn = gamepad1.right_stick_x;
                double multiplier = slowMode ? 0.3 : 1;
                robot.drive.mecanumDrive(power * multiplier, strafe * multiplier, turn * multiplier);
            } else {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double botHeading = -poseEstimate.getHeading();
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                double multiplier = slowMode ? 0.3 : 1;
                robot.drive.setDrivePowers(frontLeftPower * multiplier, frontRightPower * multiplier, backLeftPower * multiplier, backRightPower * multiplier);

            }
            tel.addData("Drive", Config.fieldCentric);
            tel.addData("Pose", poseEstimate);
            tel.addData("Lift: ", robot.lift.leftLift.getCurrentPosition());
            tel.addData("SlowMode: ", slowMode);
            tel.addData("Time: ", timer.seconds());

            tel.addData("pX: ", poseEstimate.getX());
            tel.addData("pY: ", poseEstimate.getY());
            tel.addData("pH: ", poseEstimate.getRotation().getDegrees());
            telemetry.addData("pose", drive.getPose().toString());
//            if (lastAprilTagPos != null) {
//                timer.reset();
//                tel.addData("atp pos", lastAprilTagPos.toString());
//                while (timer.milliseconds() < 2000 || !movement.move(poseEstimate, lastAprilTagPos, tel)) {
//                }
//                lastAprilTagPos = null;
//                robot.drive.setDrivePowers(0, 0, 0, 0);
//            }

//            if (path != null) {
//                if (!movement.move(poseEstimate, path[0], tel)) {
//                    robot.drive.setDrivePowers(0, 0, 0, 0);
//                    telemetry.addData("Done", "done");
//                    telemetry.update();
//                    path = null;
//                }
//            }
            telemetry.update();
            drive.updatePoseEstimate();

            tel.update();
            tel.update();
        }
    }
}