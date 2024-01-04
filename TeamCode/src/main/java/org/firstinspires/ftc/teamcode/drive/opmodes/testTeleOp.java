package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AprilTags.AprilTagsInit;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

@TeleOp(name = "Test", group = "Linear Opmode")
public class testTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        Robot robot = new Robot(hardwareMap, gamepad1);
        while (opModeIsActive()) {
            if(gamepad1.x)
            {
                robot.drive.backLeft.setPower(0.5);
            }
            if(gamepad1.y)
            {
                robot.drive.backRight.setPower(0.5);
            }
            if(gamepad1.a)
            {
                robot.drive.frontLeft.setPower(0.5);
            }
            if(gamepad1.b)
            {
                robot.drive.frontRight.setPower(0.5);
            }
//            if (gamepad1.y) {
//            } else if (gamepad1.x) {
//                robot.claw.topServo.setPosition(Config.topServoClose);
//            } else if (gamepad1.a) {
//                robot.claw.bottomServo.setPosition(Config.bottomServoOpen);
//            } else if (gamepad1.b) {
//                robot.claw.bottomServo.setPosition(Config.bottomServoClose);
//            }

//            if (gamepad1.dpad_left) {
//                robot.hang.hangMotor.setPower(0.8);
//            } if (gamepad1.dpad_right) {
//                robot.hang.hangMotor.setPower(-0.8);
//            }

            telemetry.update();
        }
    }
}
