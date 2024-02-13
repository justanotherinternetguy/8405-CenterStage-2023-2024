package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "PIDTuning", group = "Linear Opmode")
public class PIDTuning extends LinearOpMode {
    public static boolean fieldCentric = false;
    public static boolean slowMode = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Movement movement = new Movement(robot.drive);
        Pose2d[] path = null;
        Telemetry dashTel = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
//            robot.lift.liftToBase();
            if (path == null) {
                if (gamepad1.y) {
                    path = new Pose2d[] {
                            createPose(0, 24, 0)
                    };
                } else if (gamepad1.x) {
                    path = new Pose2d[] {
                            createPose(24, 0, 0)
                    };
                } else if (gamepad1.b) {
                    path = new Pose2d[] {
                            createPose(-24, 0, 0)
                    };
                } else if (gamepad1.a) {
                    path = new Pose2d[] {
                            createPose(0, -24, 0)
                    };
                } else if (gamepad1.right_bumper) {
                    path = new Pose2d[] {
                            createPose(0, 0, 0)
                    };
                }
                continue;
            }
            rrDrive.updatePoseEstimate();
            Pose2d pose = rrDrive.getPose();
            dashTel.addData("pose", pose);
            if (!movement.move(pose, path[0], dashTel)) {
                robot.drive.setDrivePowers(0, 0, 0, 0);
                dashTel.addData("Done", "done");
                path = null;
            } else {
                dashTel.addData("pose x", pose.getX());
                dashTel.addData("pose y", pose.getY());
                dashTel.addData("heading", pose.getRotation().getDegrees());
            }
            dashTel.update();
        }
    }



    public Pose2d createPose(double x, double y, double heading) {
        return new Pose2d(x, y, new Rotation2d(Math.toRadians(heading)));
    }
}