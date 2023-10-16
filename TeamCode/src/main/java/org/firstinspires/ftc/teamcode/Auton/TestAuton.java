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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class TestAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Odometry odometry = new Odometry(hardwareMap, robot.drive.imu);
//        Movement movement = new Movement(robot.drive, odometry, this::opModeIsActive, new PID.Config(.11, 0, 0), new PID.Config(0.05, 0, 0), 3, telemetry);
        Movement movement = new Movement(robot.drive, rrDrive, this::opModeIsActive, new PID.Config(0.1, 0, 0.05), new PID.Config(0.065, 0, 0), 0.25, telemetry);
        waitForStart();
        odometry.reset();
        robot.drive.imu.resetYaw();
        if (opModeIsActive()) {
            for (int i = 0; i < 200; i++) {
                if (!opModeIsActive()) return;
                sleep(10);
                rrDrive.update();
            }
            movement.move(new Pose2d(0, 24, new Rotation2d(Math.toRadians(0))));
        }
        while (opModeIsActive()) {
            telemetry.addData("Pose", odometry.getPose().toString());
            telemetry.update();
        }
    }
}
