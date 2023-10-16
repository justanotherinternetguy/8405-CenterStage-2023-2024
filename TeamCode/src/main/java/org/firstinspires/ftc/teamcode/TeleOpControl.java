package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="Mecanum Drive", group="Linear Opmode")
public class TeleOpControl extends LinearOpMode {
    public static boolean fieldCentric = true;
    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        robot.drive.imu.resetYaw();
        timer.reset();

        while (opModeIsActive()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("left x: ", gamepad1.left_stick_x);
            telemetry.addData("left y: ", gamepad1.left_stick_y);
            telemetry.addData("right x: ", gamepad1.right_stick_x);
            telemetry.addData("right y: ", gamepad1.right_stick_y);

            if (!fieldCentric) {
                double power = -gamepad1.left_stick_y; // remember this is reversed
                double strafe = gamepad1.left_stick_x * 1.1; // counteract imperfect strafing
                double turn = gamepad1.right_stick_x;
                robot.drive.mecanumDrive(power, strafe, turn);
                telemetry.addData("drive: ", "robotCentric");
            } else {
                double elapsed = timer.seconds();

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double h = gamepad1.right_stick_x;

                double profileY = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, y, elapsed);
                double profileX = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, x, elapsed);
                double rx = MotionProfile.motion_profile(Odometry.MAX_ACCEL * 4, Odometry.MAX_VELOCITY * 4, h, elapsed);

//                double botHeading = Math.toRadians(-odometry.getHeading());
                double botHeading = poseEstimate.getHeading();
//                double botHeading = Math.toRadians(nextHeading);

//                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
////              double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                double rotX = profileX * Math.cos(-botHeading) - profileY * Math.sin(-botHeading);
                double rotY = profileX * Math.sin(-botHeading) + profileY * Math.cos(-botHeading);

                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                robot.drive.setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
                telemetry.addData("drive: ", "fieldCentric");
            }

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}