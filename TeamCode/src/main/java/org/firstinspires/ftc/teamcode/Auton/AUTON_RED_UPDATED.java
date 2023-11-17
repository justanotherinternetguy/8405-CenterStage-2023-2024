package org.firstinspires.ftc.teamcode.Auton;

import androidx.appcompat.widget.AppCompatCheckedTextView;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.ObjectDet.ObjectDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auton.Actor.ActionInput;
import org.firstinspires.ftc.teamcode.Auton.Actor.ActionInput.inputType;

@Autonomous
public class AUTON_RED_UPDATED extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ObjectDetector teamPropDet;
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Odometry odometry = new Odometry(hardwareMap, robot.drive.imu);
//        AprilTagsInit init;
        PID.Config translationConfig = new PID.Config(Config.translationP, Config.translationI, Config.translationD);
        PID.Config rotationConfig = new PID.Config(Config.rotationP, Config.rotationI, Config.rotationD);
        Movement movement = new Movement(robot.drive, rrDrive, this::opModeIsActive, translationConfig, rotationConfig, Config.tolerance, Config.toleranceH, telemetry);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        teamPropDet = new ObjectDetector(hardwareMap, tel);
//        robot.claw.setPower(Config.bottomServoClose, Config.topServoClose);
        waitForStart();
        odometry.reset();
        robot.drive.imu.resetYaw();
        int[] objectCenter = teamPropDet.search();
        double third = 1920.0/3+100; // middle of camera, change later
        int direction = -1;

        inputType Lift = inputType.LIFT;
        inputType Claw = inputType.CLAW;
        inputType Mvmt = inputType.MOVEMENT;
        Actor actor = new Actor(hardwareMap, telemetry, robot, rrDrive, 5000);
        if (objectCenter != null) {
            int centerX = objectCenter[0];
            if (centerX < third) { // left
                direction = 0;
                actor.add(new ActionInput(Mvmt, new int[] {0, 28, 0, 80}), false);
                actor.add(new ActionInput(Claw, new int[] {0}), true);
            }
            else if (centerX > 2 * third) { // right
                direction = 1;
                actor.add(new ActionInput(Mvmt, new int[] {0, 28, 0, 80}), false);
                actor.add(new ActionInput(Mvmt, new int[] {0, -28, 0, 80}), true);

            }
            else {
                direction = 2;
                actor.add(new ActionInput(Mvmt, new int[] {0, 28, 0, 80}), false);
                actor.add(new ActionInput(Claw, new int[] {0}), true);

            }
        }


        tel.addData("lift", 0);
        tel.addData("liftPower", 0);
        tel.addData("liftTarget", 0);
        tel.update();
        while (opModeIsActive()) {
            actor.act();
            tel.addData("objectcenter: ", objectCenter[0]);
            tel.addData("direction", direction);
            tel.addData("lift", robot.lift.encoder.getCurrentPosition());
            tel.addData("Pose", odometry.getPose().toString());
            tel.update();
        }
    }
}