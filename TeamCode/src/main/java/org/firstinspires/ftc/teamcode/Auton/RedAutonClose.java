package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Actor.Actor;
import org.firstinspires.ftc.teamcode.Control.Actor.ClawAction;
import org.firstinspires.ftc.teamcode.Control.Actor.MvntAction;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RedAutonClose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        Movement movement = new Movement(robot.drive);
        Actor actor = new Actor(hardwareMap, telemetry, robot, rrDrive, movement, 3000);

        rrDrive.setPoseEstimate(new Pose2d(0, 0, 0));

        actor.add(new MvntAction(createPose(0, 5, 0)))
                .add(new ClawAction(0.5))
                .add(new MvntAction(createPose(0, 0, 0)));

        boolean pathDone = false;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            System.out.println(actor.actions);
            if (pathDone) {
                tel.addData("State", "Done");
                tel.update();
                continue;
            }
            int pathRemaining = actor.run();
            if (pathRemaining == 0) {
                pathDone = true;
            }
            tel.addData("State", pathRemaining);
            tel.update();
        }
    }

    public com.arcrobotics.ftclib.geometry.Pose2d createPose(double x, double y, double heading) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(x, y, new Rotation2d(Math.toRadians(heading)));
    }
}
