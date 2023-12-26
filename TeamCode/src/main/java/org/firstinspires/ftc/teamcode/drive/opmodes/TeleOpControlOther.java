package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "Drive Testing", group = "Linear Opmode")
public class TeleOpControlOther extends LinearOpMode {
    public static boolean slowMode = false;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime timer = new ElapsedTime();

//        AprilTagsInit apriltags = new AprilTagsInit(hardwareMap, telemetry);
//        apriltags.initialize(telemetry);

//        TwoWheelTrackingLocalizer tw = new TwoWheelTrackingLocalizer(hardwareMap, drive);

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();

        boolean lastX = false;

        waitForStart();

        robot.drive.imu.resetYaw();
        robot.odom.reset();
        timer.reset();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d poseEstimate = drive.getPose();

            if (gamepad1.x && !lastX) {
                slowMode = !slowMode;
            }
            lastX = gamepad1.x;
            robot.hang.input(gamepad1);
            robot.claw.input(gamepad1);
            robot.lift.liftTeleOp(gamepad1, tel);

            if (!Config.fieldCentric) {
                double power = -gamepad1.left_stick_y; // remember this is reversed
                double strafe = gamepad1.left_stick_x * 1.5; // counteract imperfect strafing
                double turn = gamepad1.right_stick_x;
                double multiplier = slowMode ? 0.3 : 1;
                tel.addData("power: ", power);
                tel.addData("strafe: ", strafe);
                tel.addData("turn: ", turn);
                robot.drive.mecanumDrive(power * multiplier, strafe * multiplier, turn * multiplier);
            } else {
                double elapsed = timer.seconds();
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double h = gamepad1.right_stick_x;

                double profileY = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, y, elapsed);
                double profileX = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, x, elapsed);
                double rx = MotionProfile.motion_profile(Odometry.MAX_ACCEL * 4, Odometry.MAX_VELOCITY * 4, h, elapsed);

                double botHeading = -poseEstimate.getHeading();
                double rotX = profileX * Math.cos(-botHeading) - profileY * Math.sin(-botHeading);
                double rotY = profileX * Math.sin(-botHeading) + profileY * Math.cos(-botHeading);

                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                double multiplier = slowMode ? 0.3 : 1;
                robot.drive.setDrivePowers(frontLeftPower * multiplier, frontRightPower * multiplier, backLeftPower * multiplier, backRightPower * multiplier);
                telemetry.addData("drive: ", "fieldCentric");

            }
            tel.addData("drive", Config.fieldCentric);
//            tel.addData("TEMP", apriltags.camera.getWhiteBalanceControl().getWhiteBalanceTemperature());
            tel.addData("pose", poseEstimate);
            tel.addData("pose ex", drive.getPose().toString());
            tel.addData("!LEFT ENCODER: ", robot.odom.getEncoders()[0]);
            tel.addData("!RIGHT ENCODER: ", robot.odom.getEncoders()[1]);
            tel.addData("!CENTER ENCODER: ", robot.odom.getEncoders()[2]);

            tel.update();

//            telemetry.addData("maxfps", apriltags.camera.getCurrentPipelineMaxFps());
//            telemetry.addData("fps", apriltags.camera.getFps());
            telemetry.addData("SlowMode: ", slowMode);

//            telemetry.addData("top claw", robot.claw.timer.milliseconds());

            telemetry.update();
        }
    }
}