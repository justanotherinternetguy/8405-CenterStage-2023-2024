package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Subsystems.*;


@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class TeleOpControl extends LinearOpMode {

    // Declare OpMode members.

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        // run until the end of the match (driver presses STOP)
        waitForStart();


        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y; // remember this is reversed
            robot.drive.moveTest(power);

            telemetry.addData("left x: ", gamepad1.left_stick_x);
            telemetry.addData("left y: ", gamepad1.left_stick_y);
            telemetry.addData("right x: ", gamepad1.right_stick_x);
            telemetry.addData("right y: ", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}