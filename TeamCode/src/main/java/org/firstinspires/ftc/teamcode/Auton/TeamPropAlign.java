package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AprilTags.AprilTagsInit;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Control.Rotate;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.ObjectDet.ObjectDetector;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class TeamPropAlign extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ObjectDetector teamPropDet;
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Odometry odometry = new Odometry(hardwareMap, robot.drive.imu);
        AprilTagsInit init;
        PID.Config translationConfig = new PID.Config(Config.translationP, Config.translationI, Config.translationD);
        PID.Config rotationConfig = new PID.Config(Config.rotationP, Config.rotationI, Config.rotationD);
        Movement movement = new Movement(robot.drive, rrDrive, this::opModeIsActive, translationConfig, rotationConfig, Config.tolerance, telemetry);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        teamPropDet = new ObjectDetector(hardwareMap, tel);
        waitForStart();
        odometry.reset();
        robot.drive.imu.resetYaw();

        Pose2d[] paths;

        int[] objectCenter = teamPropDet.search();
        int targetX = 640; // middle of camera

        if (objectCenter != null) {
            int centerX = objectCenter[0];
            if (centerX < targetX) {
                paths = new Pose2d[]{
                    new Pose2d(-18, 0, new Rotation2d(Math.toRadians(0))),
                };
            }
            else if (centerX > targetX) {
                paths = new Pose2d[]{
                        new Pose2d(18, 0, new Rotation2d(Math.toRadians(0))),
                };
            }
            else {
                paths = new Pose2d[]{
                        new Pose2d(0, 24, new Rotation2d(Math.toRadians(0))),
                };
            }
        }


        Pose2d[] path = new Pose2d[] {
        };




        int pathOn = 0;
        while (opModeIsActive()) {
            if (pathOn != path.length) {
                if (!movement.move(path[pathOn])) {
                    pathOn++;
                    robot.drive.setDrivePowers(0, 0, 0, 0);
                }
            } else {
                robot.drive.setDrivePowers(0, 0, 0, 0);
            }
            tel.addData("path on", pathOn);
            tel.addData("Pose", odometry.getPose().toString());
            tel.update();
        }
    }
}