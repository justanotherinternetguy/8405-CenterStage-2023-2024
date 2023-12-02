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
        Claw claw = robot.claw;
        while (opModeIsActive()) {
            if(gamepad1.x)
            {
                claw.clawServo.setPosition(0.775);
            }
            if (gamepad1.y) {
                claw.clawServo.setPosition(1);
            }
            telemetry.update();
        }
        }
}
