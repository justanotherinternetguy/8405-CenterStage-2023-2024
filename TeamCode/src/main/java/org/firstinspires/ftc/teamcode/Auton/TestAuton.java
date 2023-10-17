package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
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
        PID.Config translationConfig = new PID.Config(AutonSettings.translationP, AutonSettings.translationI, AutonSettings.translationD);
        PID.Config rotationConfig = new PID.Config(AutonSettings.rotationP, AutonSettings.rotationI, AutonSettings.rotationD);
        Movement movement = new Movement(robot.drive, rrDrive, this::opModeIsActive, translationConfig, rotationConfig, 1, telemetry);
        waitForStart();
        odometry.reset();
        robot.drive.imu.resetYaw();
        if (opModeIsActive()) {
//            for (int i = 0; i < 200; i++) {
//                if (!opModeIsActive()) return;
//                sleep(10);
//                rrDrive.update();
//            }
            movement.move(new Pose2d(AutonSettings.targetX, AutonSettings.targetY, new Rotation2d(Math.toRadians(AutonSettings.targetH))));
        }
        while (opModeIsActive()) {
            telemetry.addData("Pose", odometry.getPose().toString());
            telemetry.update();
        }
    }
}

@Config
class AutonSettings {
    public static double translationP = 0.3;
    public static double translationI = 0.0;
    public static double translationD = 0.0;
    public static double rotationP = 0.03;
    public static double rotationI = 0.0;
    public static double rotationD = 0.0;

    public static double targetX = 0;
    public static double targetY = 24;
    public static double targetH = 90;
}