package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "RevampTest", group = "Linear Opmode")
public class RevampTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Movement movement = new Movement(robot.drive);
        Pose2d[] path = null;
        waitForStart();
        robot.odom.reset();
        while (opModeIsActive() && !isStopRequested()) {
            Pose2d pose = rrDrive.getPose();
            robot.drive.setDrivePowers(Drive.absoluteMovement(1, 0, 0, pose.getHeading()));
        }
    }
}