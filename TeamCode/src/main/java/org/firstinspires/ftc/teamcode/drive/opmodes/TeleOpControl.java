package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Mecanum Drive", group = "Linear Opmode")
public class TeleOpControl extends LinearOpMode {
    public boolean slowMode = false;

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

//        ObjectDetector objectDetector = new ObjectDetector(hardwareMap, tel);

        boolean lastX = false;

        waitForStart();
        robot.drive.imu.resetYaw();
        timer.reset();

        while (opModeIsActive()) {
//            int[] results = objectDetector.search();

//            for (int i = 0; i < results.length; i++) {
//                tel.addData("Results " + i, results[i]);
//            }

            drive.updatePoseEstimate();
            Pose2d poseEstimate = drive.getPose();

            robot.lift.liftTeleOp(gamepad1, tel);

            robot.claw.input(gamepad1);
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
            tel.update();
        }
    }
}