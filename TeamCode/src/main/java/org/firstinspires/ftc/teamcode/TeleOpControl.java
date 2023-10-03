package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Subsystems.*;


@TeleOp(name="Mecanum Drive", group="Linear Opmode")
public class TeleOpControl extends LinearOpMode {

    // Declare OpMode members.

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        // run until the end of the match (driver presses STOP)
        waitForStart();

        boolean fieldCentric = true;

        while (opModeIsActive()) {
            telemetry.addData("left x: ", gamepad1.left_stick_x);
            telemetry.addData("left y: ", gamepad1.left_stick_y);
            telemetry.addData("right x: ", gamepad1.right_stick_x);
            telemetry.addData("right y: ", gamepad1.right_stick_y);
            telemetry.update();
            if (!fieldCentric) {
                double power = -gamepad1.left_stick_y; // remember this is reversed
                double strafe = gamepad1.left_stick_x * 1.1; // counteract imperfect strafing
                double turn = gamepad1.right_stick_x;
                robot.drive.mecanumDrive(power, strafe, turn);
                continue;
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = 0;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.drive.frontLeft.setPower(frontLeftPower);
            robot.drive.backLeft.setPower(backLeftPower);
            robot.drive.frontRight.setPower(frontRightPower);
            robot.drive.backRight.setPower(backRightPower);
        }
    }
}