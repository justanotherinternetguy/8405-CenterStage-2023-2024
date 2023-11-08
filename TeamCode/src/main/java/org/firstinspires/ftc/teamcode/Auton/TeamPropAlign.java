package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AprilTags.AprilTagsInit;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Control.Rotate;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.ObjectDet.ObjectDetector;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
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
//        AprilTagsInit init;
        PID.Config translationConfig = new PID.Config(Config.translationP, Config.translationI, Config.translationD);
        PID.Config rotationConfig = new PID.Config(Config.rotationP, Config.rotationI, Config.rotationD);
        Movement movement = new Movement(robot.drive, rrDrive, this::opModeIsActive, translationConfig, rotationConfig, Config.tolerance, telemetry);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        teamPropDet = new ObjectDetector(hardwareMap, tel);
        robot.claw.setPower(Config.bottomServoClose, Config.topServoClose);
        waitForStart();
        odometry.reset();
        robot.drive.imu.resetYaw();

        Pose2d[] paths = new Pose2d[0];


        int[] objectCenter = teamPropDet.search();
        double third = 1920.0/3+100; // middle of camera, change later
        int direction = -1;
        if (objectCenter != null) {
            int centerX = objectCenter[0];
            if (centerX < third) { // left
                direction = 0;
                paths = new Pose2d[]{
                        new Pose2d(0, 28, new Rotation2d(Math.toRadians(0))),
                        new Pose2d(-4, 28, new Rotation2d(Math.toRadians(-90)))
                };
            }
            else if (centerX > 2 * third) { // right
                direction = 1;
                paths = new Pose2d[]{
                        new Pose2d(0, 28, new Rotation2d(Math.toRadians(0))),
                        new Pose2d(4, 28, new Rotation2d(Math.toRadians(90)))
                };
            }
            else {
                direction = 2;
                paths = new Pose2d[]{ // center
                        new Pose2d(0, 24, new Rotation2d(Math.toRadians(0))),
                };
            }
        }


        int pathOn = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            robot.lift.liftToPos(100, Config.liftMotorPowerMacro);

            if (pathOn != paths.length) {
                if (!movement.move(paths[pathOn]) || timer.seconds() > 2.5) {
                    pathOn++;
                    timer = new ElapsedTime();
                    tel.addData("current positionX: ", rrDrive.getPose().getX());
                    tel.addData("current positionY: ", rrDrive.getPose().getY());
                    robot.drive.setDrivePowers(0, 0, 0, 0);
                }

            } else {
                robot.claw.setPower(Config.bottomServoOpen, Config.topServoClose);
                robot.drive.setDrivePowers(0, 0, 0, 0);
            }
            tel.addData("objectcenter: ", objectCenter[0]);
            tel.addData("direction", direction);
            tel.addData("path on", pathOn);
            tel.addData("Pose", odometry.getPose().toString());
            tel.update();
        }
    }

}