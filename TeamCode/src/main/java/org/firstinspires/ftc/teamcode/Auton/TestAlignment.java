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
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

@Autonomous
public class TestAlignment extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        AprilTagsInit apriltags = new AprilTagsInit(hardwareMap, telemetry);
        apriltags.initialize();
        int apriltag_x = 960;
        int apriltag_y = 400;
        int id = 1;
        int thres = 6;

        Odometry odometry = new Odometry(hardwareMap, robot.drive.imu);
//        Movement movement = new Movement(robot.drive, odometry, this::opModeIsActive, new PID.Config(.11, 0, 0), new PID.Config(0.05, 0, 0), 3, telemetry);
        PID.Config translationConfig = new PID.Config(Config.translationP, Config.translationI, Config.translationD);
        PID.Config rotationConfig = new PID.Config(Config.rotationP/1.5, Config.rotationI, Config.rotationD);
        Movement movement = new Movement(robot.drive, rrDrive, translationConfig, rotationConfig, 1, telemetry);
        waitForStart();
        odometry.reset();
        robot.drive.imu.resetYaw();
        Pose2d initial = rrDrive.getPose();
        boolean isAligned = false;
        double[] lastPos = null;

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        while (opModeIsActive()) {

            if(!isAligned)
            {
                double[] pos = apriltags.searchFor(id);
                //Telemetry tel = FtcDashboard.getInstance().getTelemetry();
                tel.addData("pos", Arrays.toString(pos));
                tel.addData("diff", pos != null ? pos[1] - apriltag_x : "none");
                if(pos == null && lastPos != null)
                {
                    pos = lastPos;
                }
                if(pos != null) {
                    tel.addData("X: ", pos[1]);
                    if (pos[1] - apriltag_x < -thres)
                    {
//                        movement.strafeAt(Config.alignment, initial, Movement.DIRECTION.LEFT);
                        movement.fieldCentricMove(new Pose2d(-Config.alignment, initial.getY(), initial.getRotation()), 1);
                    }
                    else if(pos[1] - apriltag_x > thres)
                    {
//                        movement.strafeAt(Config.alignment, initial, Movement.DIRECTION.RIGHT);
                        movement.fieldCentricMove(new Pose2d(Config.alignment, initial.getY(), initial.getRotation()), 1);
                    }
                    else
                    {
                        isAligned = true;
                        tel.addLine("DONE!");
                    }
                    lastPos = pos;
                    tel.update();
                }
            }
        }

    }
}