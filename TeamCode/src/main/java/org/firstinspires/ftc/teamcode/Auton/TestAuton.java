package org.firstinspires.ftc.teamcode.Auton;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Control.Rotate;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class TestAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Odometry odometry = new Odometry(hardwareMap, robot.drive.imu);
//        Movement movement = new Movement(robot.drive, odometry, this::opModeIsActive, new PID.Config(.11, 0, 0), new PID.Config(0.05, 0, 0), 3, telemetry);
        PID.Config translationConfig = new PID.Config(Config.translationP, Config.translationI, Config.translationD);
        PID.Config rotationConfig = new PID.Config(Config.rotationP, Config.rotationI, Config.rotationD);
        Movement movement = new Movement(robot.drive, rrDrive, this::opModeIsActive, translationConfig, rotationConfig, 1, telemetry);
        waitForStart();
        odometry.reset();
        robot.drive.imu.resetYaw();
        if (opModeIsActive()) {
            movement.move(new Pose2d(Config.targetX, Config.targetY, new Rotation2d(Math.toRadians(Config.targetH))));
            for (int i = 0; i < Config.pathCount; i ++) {
                movement.move(new Pose2d(0, 24 * 2.125, new Rotation2d(Math.toRadians(90))));
                movement.move(new Pose2d(24 * -2, 24 * 2.125, new Rotation2d(Math.toRadians(90))));
                movement.move(new Pose2d(24 * -3.25, 24 * 1.125, new Rotation2d(Math.toRadians(-90))));
                movement.move(new Pose2d(24 * -2, 24 * 2.125, new Rotation2d(Math.toRadians(-90))));
                movement.move(new Pose2d(0, 24 * 2.125, new Rotation2d(Math.toRadians(-90))));
                movement.move(new Pose2d(24 * 0.75, 24 * 1.5, new Rotation2d(Math.toRadians(90))));
            }
        }
        while (opModeIsActive()) {
            telemetry.addData("Pose", odometry.getPose().toString());
            telemetry.update();
        }
    }
}