package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.Subsystems.*;


@TeleOp(name="Mecanum Drive", group="Linear Opmode")
public class TeleOpControl extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        Odometry odometry = new Odometry(hardwareMap, robot.drive.imu);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        boolean fieldCentric = true;

        odometry.reset();
        robot.drive.imu.resetYaw();
        timer.reset();

        double lastMs = 0;
        double lastIMU = 0;

        while (opModeIsActive()) {
            telemetry.addData("left x: ", gamepad1.left_stick_x);
            telemetry.addData("left y: ", gamepad1.left_stick_y);
            telemetry.addData("right x: ", gamepad1.right_stick_x);
            telemetry.addData("right y: ", gamepad1.right_stick_y);
            double imuValue = robot.drive.getIMU();

            double delta = timer.milliseconds() - lastMs;
            double nextHeading = lastIMU + robot.drive.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * delta / 1000;


            if (!fieldCentric) {
                double power = -gamepad1.left_stick_y; // remember this is reversed
                double strafe = gamepad1.left_stick_x * 1.1; // counteract imperfect strafing
                double turn = gamepad1.right_stick_x;
                robot.drive.mecanumDrive(power, strafe, turn);
                telemetry.addData("drive: ", "robotCentric");
            } else {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double h = gamepad1.right_stick_x;

                double elapsed = timer.seconds();
                double profileY = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, y, elapsed);
                double profileX = MotionProfile.motion_profile(Odometry.MAX_ACCEL, Odometry.MAX_VELOCITY, x, elapsed);
                double rx = MotionProfile.motion_profile(Odometry.MAX_ACCEL * 4, Odometry.MAX_VELOCITY * 4, h, elapsed);

//                double botHeading = Math.toRadians(-odometry.getHeading());
                double botHeading = Math.toRadians(lastIMU == imuValue ? nextHeading : imuValue);
                telemetry.addData("WHAT: ", botHeading);

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

            lastMs = timer.milliseconds();
            lastIMU = robot.drive.getIMU();

            odometry.update();
            telemetry.addData("l: ", odometry.getEncoders()[0]);
            telemetry.addData("r: ", odometry.getEncoders()[1]);
            telemetry.addData("c: ", odometry.getEncoders()[2]);
            telemetry.addData("pose: ", odometry.getPose().toString());
            telemetry.addData("x: ", odometry.getX());
            telemetry.addData("y: ", odometry.getY());
            telemetry.addData("heading: ", odometry.getHeading());
            telemetry.addData("heading IMU: ", imuValue);
            telemetry.addData("PREDICTION IMU: ", nextHeading);
            telemetry.update();
        }
    }
}