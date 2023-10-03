package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.jvm.Gen;

import org.firstinspires.ftc.teamcode.Subsystems.*;


@TeleOp(name="Mecanum Drive", group="Linear Opmode")
public class TeleOpControl extends LinearOpMode {

    // Declare OpMode members.

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        Odometry odometry = new Odometry(hardwareMap);
        // run until the end of the match (driver presses STOP)
        waitForStart();

        boolean fieldCentric = false;

        odometry.reset();

        while (opModeIsActive()) {
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
                telemetry.update();
                continue;
            } else {

                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double botHeading = Math.toRadians(-odometry.getHeading());

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                robot.drive.setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
                telemetry.addData("drive: ", "fieldCentric");
            }

            odometry.update();
            telemetry.addData("l: ", odometry.getEncoders()[0]);
            telemetry.addData("r: ", odometry.getEncoders()[1]);
            telemetry.addData("c: ", odometry.getEncoders()[2]);
            telemetry.addData("pose: ", odometry.getPose().toString());
            telemetry.addData("x: ", odometry.getX());
            telemetry.addData("y: ", odometry.getY());
            telemetry.addData("heading: ", odometry.getHeading());
//            telemetry.addData("drive: ", "fieldCentric");
            telemetry.update();
        }
    }
}