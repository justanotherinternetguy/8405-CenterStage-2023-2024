package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Control.Actor.Actor;
import org.firstinspires.ftc.teamcode.Control.Actor.MvntAction;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "PIDTuning", group = "Linear Opmode")
public class PIDTuning extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Movement movement = new Movement(robot.drive);
        Pose2d[] path = null;
        Telemetry dashTel = FtcDashboard.getInstance().getTelemetry();
        Actor actor = new Actor(hardwareMap, telemetry, robot, rrDrive, movement, 3000);

        rrDrive.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(0, 0, 0));

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.lift.liftToPos(Config.FLOOR, Config.liftMotorPowerAuton);
            rrDrive.update();
            Pose2d pose = rrDrive.getPose();
            dashTel.addData("pose x", pose.getX());
            dashTel.addData("pose y", pose.getY());
            dashTel.addData("heading", pose.getRotation().getDegrees());
            if (gamepad1.y) {
                actor.add(new MvntAction(createPose(0, 24, 0)), Double.POSITIVE_INFINITY);
            } else if (gamepad1.x) {
                actor.add(new MvntAction(createPose(-24, 0, 0)), Double.POSITIVE_INFINITY);
            } else if (gamepad1.b) {
                actor.add(new MvntAction(createPose(24, 0, 0)), Double.POSITIVE_INFINITY);
            } else if (gamepad1.a) {
                actor.add(new MvntAction(createPose(0, -24, 0)), Double.POSITIVE_INFINITY);
            } else if (gamepad1.dpad_right) {
                actor.add(new MvntAction(createPose(0, 0, 90)), Double.POSITIVE_INFINITY);
            } else if (gamepad1.dpad_down) {
                actor.add(new MvntAction(createPose(0, 0, 180)), Double.POSITIVE_INFINITY);
            } else if (gamepad1.dpad_left) {
                actor.add(new MvntAction(createPose(0, 0, -90)), Double.POSITIVE_INFINITY);
            } else if (gamepad1.right_bumper) {
                actor.add(new MvntAction(createPose(0, 0, 0)), Double.POSITIVE_INFINITY);
            }
            dashTel.addData("pose", pose);
            int remaining = actor.run();
            if (remaining == 0) {
                dashTel.addData("status", "done");
                robot.drive.setDrivePowers(0,0,0,0);
            } else {
                dashTel.addData("status", "moving");
            }
            dashTel.update();
        }
    }


    public Pose2d createPose(double x, double y, double heading) {
        return new Pose2d(x, y, new Rotation2d(Math.toRadians(heading)));
    }
}