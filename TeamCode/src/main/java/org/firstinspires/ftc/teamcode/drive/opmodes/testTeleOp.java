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
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "Test", group = "Linear Opmode")
public class testTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();

        Robot robot = new Robot(hardwareMap, gamepad1);
        while (opModeIsActive()) {
//            if (gamepad1.right_trigger > 0.2 || gamepad1.left_trigger > 0.2)    robot.lift.liftManual(gamepad1, telemetry);
//            else robot.lift.setLiftPower(0);
//

            tel.addData("OUTPUT:", robot.lift.liftToPos(400, 0.8));
            tel.update();


            //            if(gamepad1.a)
//            {
//                robot.drive.frontLeft.setPower(0.5);
//            }
//            if(gamepad1.b)
//            {
//                robot.drive.frontRight.setPower(0.5);
//            }
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

            telemetry.addData("left", robot.lift.leftLift.getCurrentPosition());
            telemetry.addData("right", robot.lift.rightLift.getCurrentPosition());
            telemetry.addData("leftp", robot.lift.leftLift.getPower());
            telemetry.addData("rightp", robot.lift.rightLift.getPower());
            telemetry.addData("enc", robot.lift.encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
