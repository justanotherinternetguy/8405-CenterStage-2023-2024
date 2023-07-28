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


        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y; // remember this is reversed
            double strafe = gamepad1.left_stick_x * 1.1; // counteract imperfect strafing
            double turn = gamepad1.right_stick_x;
            robot.drive.mecanumDrive(power, strafe, turn);

            telemetry.addData("left x: ", gamepad1.left_stick_x);
            telemetry.addData("left y: ", gamepad1.left_stick_y);
            telemetry.addData("right x: ", gamepad1.right_stick_x);
            telemetry.addData("right y: ", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}