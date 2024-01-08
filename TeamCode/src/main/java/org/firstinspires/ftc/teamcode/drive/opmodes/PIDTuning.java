package org.firstinspires.ftc.teamcode.drive.opmodes;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
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
        waitForStart();
        robot.odom.reset();
        while (opModeIsActive() && !isStopRequested()) {
//            robot.lift.liftToBase();
            if (path == null) {
                if (gamepad1.x) {
                    path = new Pose2d[]{
                            new Pose2d(0, 24, new Rotation2d(Math.toRadians(0))),
                    };
                }
                else if (gamepad1.y) {
                    path = new Pose2d[]{
                            new Pose2d(0, -24, new Rotation2d(Math.toRadians(0))),
                    };
                } else if (gamepad1.a) {
                    path = new Pose2d[]{
                            new Pose2d(24, 0, new Rotation2d(0)),
                    };
                } else if (gamepad1.b) {
                    path = new Pose2d[]{
                            new Pose2d(-24, 0, new Rotation2d(Math.toRadians(0))),
                    };
                } else if (gamepad1.dpad_up) {
                    path = new Pose2d[]{
                            new Pose2d(0, 0, new Rotation2d(Math.toRadians(90)))
                    };
                } else if (gamepad1.right_bumper) {
                    path = new Pose2d[]{
                            new Pose2d(0, 0, new Rotation2d(Math.toRadians(180)))
                    };
                } else if (gamepad1.left_trigger > 0.5) {
                    path = new Pose2d[]{
                            new Pose2d(24, 24, new Rotation2d(0))
                    };
                } else if (gamepad1.right_trigger > 0.5) {
                    path = new Pose2d[]{
                            new Pose2d(0, 0, new Rotation2d(0)),
                    };
                }
                continue;
            }
            telemetry.addData("pose", rrDrive.getPose().toString());
            telemetry.addData("!LEFT ENCODER: ", robot.odom.getEncoders()[0]);
            telemetry.addData("!RIGHT ENCODER: ", robot.odom.getEncoders()[1]);
            telemetry.addData("!CENTER ENCODER: ", robot.odom.getEncoders()[2]);
            telemetry.update();
            Pose2d pose = rrDrive.getPose();
            if (path != null && !movement.move(path[0], pose)) {
                robot.drive.setDrivePowers(0, 0, 0, 0);
                telemetry.addData("Done", "done");
                telemetry.update();
                path = null;
            }
        }
    }
}