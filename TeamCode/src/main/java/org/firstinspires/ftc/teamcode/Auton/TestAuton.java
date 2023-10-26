package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        PID.Config translationConfig = new PID.Config(Config.translationP, Config.translationI, Config.translationD);
        PID.Config rotationConfig = new PID.Config(Config.rotationP, Config.rotationI, Config.rotationD);
        Movement movement = new Movement(robot.drive, rrDrive, this::opModeIsActive, translationConfig, rotationConfig, Config.tolerance, telemetry);
        ElapsedTime timer;
        waitForStart();
        odometry.reset();
        robot.drive.imu.resetYaw();

        Pose2d[] path = new Pose2d[] {
            new Pose2d(0, 24 * 2.125, new Rotation2d(Math.toRadians(90))),
            new Pose2d(24 * -2, 24 * 2.125, new Rotation2d(Math.toRadians(90))),
//            new Pose2d(24 * -3.25, 24 * 1.125, new Rotation2d(Math.toRadians(-90))),
//            new Pose2d(24 * -2, 24 * 2.125, new Rotation2d(Math.toRadians(-90))),
//            new Pose2d(0, 24 * 2.125, new Rotation2d(Math.toRadians(-90))),
//            new Pose2d(24 * 0.75, 24 * 1.5, new Rotation2d(Math.toRadians(90)))
        };

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();

        int currentWaypoint = 0;

        timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive()){
            rrDrive.update();
            Pose2d pose = rrDrive.getPose();
            if (movement.goTo(path[currentWaypoint])) {
                currentWaypoint++;
                timer.reset();
                if (currentWaypoint >= path.length) return;
//                if (currentWaypoint >= path.length) currentWaypoint = 0;
            }
            tel.addData("time", this.time);
            tel.addData("Pose", odometry.getPose().toString());
            tel.addData("at", currentWaypoint);
            telemetry.update();
        }
    }
}