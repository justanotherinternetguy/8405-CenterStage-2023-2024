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
//            for (AprilTagDetection detection : detectionList) {
//                if (detection.id == 5 && gamepad1.dpad_left) {
//                    pos = detection.ftcPose;
//
////                tel.addData(String.valueOf(detection.id), "x: " + pos.x + ", y: " + pos.y + ", z:" + pos.z + ", bearing: " + pos.bearing + ", elevation:" + pos.elevation + ", pitch: " + pos.pitch + ", range: " + pos.range + ", roll: " + pos.roll + ", yaw: " + pos.yaw);
//                    tel.addData("ID: ", String.valueOf(detection.id));
//                    tel.addData("x: ", pos.x);
//                    tel.addData("y: ", pos.y);
//                    tel.addData("z: ", pos.z);
//                    tel.addData("bearing: ", pos.bearing);
//                    tel.addData("elevation: ", pos.elevation);
//                    tel.addData("range: ", pos.range);
//                    tel.addData("pitch: ", pos.pitch);
//                    tel.addData("roll: ", pos.roll);
//                    tel.addData("yaw: ", pos.yaw);
//                    tel.addData("det center x", detection.center.x);
//                    tel.addData("det center y", detection.center.y);
//                    double cornerAngle = 90 - Math.abs(pos.yaw + pos.bearing);
////                double cornerAngle = 90 - pos.yaw - pos.bearing; // abs x might already be negative from cosine i'll check like later today or smt
//                    double flatRange = Math.sqrt(Math.pow(pos.range, 2) - Math.pow(pos.z, 2));
//                    double absoluteX = Math.cos(Math.toRadians(cornerAngle)) * flatRange;
//                    double absoluteY = Math.sin(Math.toRadians(cornerAngle)) * flatRange;
//
//                    // calculate if x is positive or negative
//                    // if(90+bearing+yaw > 90) + else -
//                    if (cornerAngle < 90) absoluteX = absoluteX * -1;
//
//                    double newX = poseEstimate.getX() + absoluteX;
//                    double newY = poseEstimate.getY() + absoluteY;
//                    double newHeading = poseEstimate.getRotation().getDegrees() - pos.yaw;

//                    double required_yaw = -pos.yaw;
//                    double required_pitch = pos.pitch;
//                    double newHeading = poseEstimate.getHeading();
//                    double move_x_relative = pos.x;
//                    double move_y_relative = pos.y;
//
//                    double delta_x = pos.x - poseEstimate.getX();
//                    double delta_y = pos.y - poseEstimate.getY();
//                    required_yaw = Math.atan2(delta_y, delta_x) - poseEstimate.getHeading();
//
//                    if (required_yaw > Math.toRadians(180)) {
//                        required_yaw -= Math.toRadians(360);
//                    }
//                    else if (required_yaw < -Math.toRadians(180)) {
//                        required_yaw += Math.toRadians(360);
//                    }
//
//                    newHeading += required_yaw;
//
//                    double move_x = move_x_relative * Math.cos(poseEstimate.getHeading()) - move_y_relative * Math.cos(poseEstimate.getHeading());
//                    double move_y = move_x_relative * Math.sin(poseEstimate.getHeading()) + move_y_relative * Math.sin(poseEstimate.getHeading());
//
//                    double final_robot_x = poseEstimate.getX() + move_x;
//                    double final_robot_y = poseEstimate.getY() + move_y;
//                    double final_robot_heading = newHeading;

//                    if ()

//                    lastAprilTagPos = new Pose2d(final_robot_x, poseEstimate.getY(), new Rotation2d((poseEstimate.getHeading()));
//                    break;
//                }
//            }

//            if (detectionList.size() == 3 && gamepad1.dpad_left) {
//                AprilTagDetection det4 = detectionList.get(0);
//                AprilTagPoseFtc pos4 = det4.ftcPose;
//                AprilTagDetection det5 = detectionList.get(1);
//                AprilTagPoseFtc pos5 = det5.ftcPose;
//                AprilTagDetection det6 = detectionList.get(2);
//                AprilTagPoseFtc pos6 = det6.ftcPose;
//
////                double move_x = pos5.x * Math.cos(poseEstimate.getHeading()) - pos5.y * Math.cos(poseEstimate.getHeading());
////                double move_y = pos5.x * Math.sin(poseEstimate.getHeading()) + pos5.y * Math.sin(poseEstimate.getHeading());
////                double final_robot_x = poseEstimate.getX() + move_x;
////                double final_robot_y = poseEstimate.getY() + move_y;
////
////                double flatRange = Math.sqrt(Math.pow(pos5.range, 2) - Math.pow(pos5.z, 2));
//                double cornerAngle = 90 - Math.abs(pos5.yaw + pos5.bearing);
////                double cornerAngle = 90 - pos.yaw - pos.bearing; // abs x might already be negative from cosine i'll check like later today or smt
//                double flatRange = Math.sqrt(Math.pow(pos5.range, 2) - Math.pow(pos5.z, 2));
//                double absoluteX = Math.cos(Math.toRadians(cornerAngle)) * flatRange;
//                double absoluteY = Math.sin(Math.toRadians(cornerAngle)) * flatRange;
//
//                // calculate if x is positive or negative
//                // if(90+bearing+yaw > 90) + else -
//                if (cornerAngle < 90) absoluteX = absoluteX * -1;
//
//                double newX = poseEstimate.getX() + absoluteX;
//                double newY = poseEstimate.getY() + absoluteY;
//                double newHeading = poseEstimate.getRotation().getDegrees() - (pos5.yaw + pos5.bearing);
//
//                tel.addData("cornerangle", cornerAngle);
//                tel.addData("flatrange", flatRange);
//                tel.addData("absx", absoluteX);
//                tel.addData("absy", absoluteY);
//                tel.addData("newX", newX);
//                tel.addData("newY", newY);
//                tel.addData("newheading", newHeading);

//                lastAprilTagPos = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), new Rotation2d(Math.toRadians(newHeading - 180)));


//                pos = detection.ftcPose;
//                tel.addData("ID: ", String.valueOf(detection.id));
//                tel.addData("x: ", pos.x);
//                tel.addData("y: ", pos.y);
//                tel.addData("z: ", pos.z);
//                tel.addData("bearing: ", pos.bearing);
//                tel.addData("elevation: ", pos.elevation);
//                tel.addData("range: ", pos.range);
//                tel.addData("pitch: ", pos.pitch);
//                tel.addData("roll: ", pos.roll);
//                tel.addData("yaw: ", pos.yaw);
//                double cornerAngle = 90 - Math.abs(pos.yaw + pos.bearing);
////                double cornerAngle = 90 - pos.yaw - pos.bearing; // abs x might already be negative from cosine i'll check like later today or smt
//                double flatRange = Math.sqrt(Math.pow(pos.range, 2) - Math.pow(pos.z, 2));
//                double absoluteX = Math.cos(Math.toRadians(cornerAngle)) * flatRange;
//                double absoluteY = Math.sin(Math.toRadians(cornerAngle)) * flatRange;
//
//                // calculate if x is positive or negative
//                // if(90+bearing+yaw > 90) + else -
//                if (cornerAngle < 90) absoluteX = absoluteX * -1;
//
//                double newX = poseEstimate.getX() + absoluteX;
//                double newY = poseEstimate.getY() + absoluteY;
//                double newHeading = poseEstimate.getRotation().getDegrees() - pos.yaw;
//
//                lastAprilTagPos = new Pose2d(final_robot_x, final_robot_y, new Rotation2d(poseEstimate.getHeading()));
//            }


//            if (detectionList.size() == 3 && gamepad1.dpad_left) {
//                AprilTagDetection det4 = detectionList.get(0);
//                AprilTagPoseFtc pos4 = det4.ftcPose;
//                AprilTagDetection det5 = detectionList.get(1);
//                AprilTagPoseFtc pos5 = det5.ftcPose;
//                AprilTagDetection det6 = detectionList.get(2);
//                AprilTagPoseFtc pos6 = det6.ftcPose;
//
//                double deltaX = poseEstimate.getX() - pos5.x;
//                tel.addData("x", pos5.x);
//                tel.addData("delta x", deltaX);
//                double deltaY = poseEstimate.getY() - pos5.y;
//                tel.addData("delta y", deltaY);
//
//                double newX = poseEstimate.getX() + pos5.x;
//                tel.addData("new X", newX);
//
////                lastAprilTagPos = new Pose2d(newX, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
////                lastAprilTagPos = new Pose2d(2, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
//                path = new Pose2d[]{
//                        new Pose2d(2, 0, new Rotation2d(0))
//                };
////                Drive.DrivePowers powers = Drive.absoluteMovement(newX, poseEstimate.getY(), h, -pose.getHeading());
////                drive.setDrivePowers(powers);
//
////                if (deltaX > 1) { //right
////                    lastAprilTagPos = new Pose2d(deltaX, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
////                }
////                if (deltaX < 1) { //right
////                    lastAprilTagPos = new Pose2d(deltaX, poseEstimate.getY(), new Rotation2d(poseEstimate.getHeading()));
////                }
//            }

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