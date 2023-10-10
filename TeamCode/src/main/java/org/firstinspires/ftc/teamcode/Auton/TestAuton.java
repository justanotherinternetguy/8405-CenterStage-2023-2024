package org.firstinspires.ftc.teamcode.Auton;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Control.Rotate;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous
public class TestAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        Odometry odometry = new Odometry(hardwareMap, robot.drive.imu);
//        Movement movement = new Movement(robot.drive, odometry, this::opModeIsActive, new PID.Config(.11, 0, 0), new PID.Config(0.05, 0, 0), 3, telemetry);
        Movement movement = new Movement(robot.drive, odometry, this::opModeIsActive, new PID.Config(.3, 0, 0), new PID.Config(0.05, 0, 0), 0.25, telemetry);
        waitForStart();
        odometry.reset();
        robot.drive.imu.resetYaw();
        if (opModeIsActive()) {
//            movement.move(new Pose2d(0, 24, new Rotation2d(Math.toRadians(0))));
//            movement.move(new Pose2d(0, -24, new Rotation2d(Math.toRadians(0))));
//            movement.move(new Pose2d(24, 24, new Rotation2d(Math.toRadians(0))));
//            movement.move(new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))));
//            movement.move(new Pose2d(0, 24, new Rotation2d(Math.toRadians(90))));
//            movement.move(new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))));
//            movement.move(new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))));
//            movement.move(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
//            movement.move(new Pose2d(24, 24, new Rotation2d(Math.toRadians(90))));
            movement.move(new Pose2d(0, 24 * 3, new Rotation2d(Math.toRadians(-90))));
            movement.move(new Pose2d(-24 * 3, 24 * 3, new Rotation2d(Math.toRadians(-180))));
            movement.move(new Pose2d(-24 * 3, 0, new Rotation2d(Math.toRadians(-270))));
            movement.move(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
        }
        while (opModeIsActive()) {
            telemetry.addData("Pose", odometry.getPose().toString());
            telemetry.update();
        }
    }
}
