package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous
public class TestAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1);
        Odometry odometry = new Odometry(hardwareMap);
        Movement movement = new Movement(robot.drive, odometry, this::opModeIsActive, 0.05, 0, 0.2, 0.3, telemetry);
        waitForStart();
        odometry.reset();
        if (opModeIsActive()) {
            movement.move(24, 0);
        }
        while (opModeIsActive()) {
            telemetry.addData("Pose", odometry.getPose().toString());
            telemetry.update();
        }
    }
}
