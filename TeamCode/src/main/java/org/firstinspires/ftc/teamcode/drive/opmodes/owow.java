package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name="motor port test", group="Linear Opmode")
public class owow extends LinearOpMode {
    public static boolean fieldCentric = false;
    public static boolean slowMode = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap,gamepad1);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.y) {
                robot.drive.frontRight.setPower(1);
            } else {
                robot.drive.frontRight.setPower(0);
            }
            if (gamepad1.x) {
                robot.drive.frontLeft.setPower(1);
            } else {
                robot.drive.frontLeft.setPower(0);
            }
            if (gamepad1.a) {
                robot.drive.backLeft.setPower(1);
            } else {
                robot.drive.backLeft.setPower(0);
            }
            if (gamepad1.b) {
                robot.drive.backRight.setPower(1);
            } else {
                robot.drive.backRight.setPower(0);
            }
        }
    }
}