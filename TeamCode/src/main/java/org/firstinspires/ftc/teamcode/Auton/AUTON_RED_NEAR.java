package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.ObjectDet.ObjectDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AUTON_RED_NEAR extends LinearOpMode {
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

        Actor path = new Actor(hardwareMap, telemetry, robot, rrDrive, movement, 2500)
                .add(new ClawAction(ClawAction.ClawStates.bottomClosed, ClawAction.ClawStates.topClosed)); // close claw

        int[] objectCenter = teamPropDet.search();
        double third = 1920.0/3+100; // middle of camera, change later
        int direction = -1;
        if (objectCenter != null) {
            int centerX = objectCenter[0];
            if (centerX < third) { // left
                direction = 0;
                path.add(new MvntAction(pose2d(0, 28, 0)))                  // drive to tape
                        .add(new MvntAction(pose2d(-3, 28, -90)))           // face the tape
                        .add(new ClawAction(ClawAction.ClawStates.bottomOpen))              // drop the purple pixel
                        .add(new MvntAction(pose2d(0, 28, -90)))            // move back and up the claw
                        .add(new LiftAction(300, Config.liftMotorPowerAuton), true)
                        .add(new MvntAction(pose2d(0, 28, 90)))             // face the board
                        .add(new MvntAction(pose2d(Config.BACKSPOT, 34.5, 90))) // move to the board and raise lifts
                        .add(new LiftAction(Config.LIFT_BACK, Config.liftMotorPowerAuton * 1.1), true)
                        .add(new MvntAction(Config.BOARDSPEED, 0, 0), 800) // drive into the wall
                        .add(new MvntAction(Config.BOARDSPEED, 0, 0), 400) // continue driving into the wall and move lift down(to tilt the pixel)
                        .add(new LiftAction(Config.LIFT_BACK - 100, Config.liftMotorPowerDown), true)
                        .add(new MvntAction(Config.BOARDSPEED, 0, 0), 200) // continue driving and drop the yellow pixel
                        .add(new ClawAction(ClawAction.ClawStates.topOpen), true)
                        .add(new MvntAction(pose2d(24, 34.5, 90)))          // move back from the board and lower lift
                        .add(new LiftAction(Config.FLOOR + 40, Config.liftMotorPowerDown * 0.8), true)
                        .add(new MvntAction(pose2d(24, 3, 90)))             // move to the wall
                        .add(new MvntAction(pose2d(45, 3, 90)));            // park
            }
            else if (centerX > 2 * third) { // right
                direction = 1;
                path.add(new MvntAction(pose2d(0, 30, 0)))                 // drive to tape
                        .add(new MvntAction(pose2d(3, 30, 90)))            // face tape
                        .add(new ClawAction(ClawAction.ClawStates.bottomOpen))             // drop purple pixel
                        .add(new MvntAction(pose2d(0, 30, 90)))            // center and raise lift
                        .add(new LiftAction(100, Config.liftMotorPowerAuton), true)
                        .add(new MvntAction(pose2d(12, 14, 90)))           // move back a tile and start raising lift
                        .add(new LiftAction(Config.LIFT_BACK, Config.liftMotorPowerAuton * 1.1), true, true)
                        .add(new MvntAction(pose2d(24, 14, 90)))           // move forward and continue raising lift
                        .add(new LiftAction(Config.LIFT_BACK, Config.liftMotorPowerAuton * 1.1), true, true)
                        .add(new MvntAction(pose2d(Config.BACKSPOT, 22, 90)))  // move to the board and continue raising lift
                        .add(new LiftAction(Config.LIFT_BACK, Config.liftMotorPowerAuton * 1.1), true, true)
                        .add(new MvntAction(Config.BOARDSPEED, 0, 0), 800) // drive into the wall
                        .add(new MvntAction(Config.BOARDSPEED, 0, 0), 400) // continue driving into the wall and move lift down(to tilt the pixel)
                        .add(new LiftAction(Config.LIFT_BACK - 100, Config.liftMotorPowerDown), true)
                        .add(new MvntAction(Config.BOARDSPEED, 0, 0), 200) // continue driving and drop the yellow pixel
                        .add(new ClawAction(ClawAction.ClawStates.topOpen), true)
                        .add(new MvntAction(pose2d(24, 22, 90)))  // move back from the board and lower lift
                        .add(new LiftAction(Config.FLOOR + 40, Config.liftMotorPowerDown * 0.8), true)
                        .add(new MvntAction(pose2d(24, 3, 90)))             // move to the wall
                        .add(new MvntAction(pose2d(45, 3, 90)));            // park
            }
            else {
                direction = 2;
                path.add(new MvntAction(pose2d(0, 28, 0)))                   // move to tape
                        .add(new ClawAction(ClawAction.ClawStates.bottomOpen))               // drop purple pixel
                        .add(new LiftAction(300, Config.liftMotorPowerAuton))        // lift up height a bit while also moving back(to not knock over the purple pixel)
                        .add(new MvntAction(pose2d(0, 26, 0)), true)
                        .add(new MvntAction(pose2d(0, 27.5, 90)))            // rotate towards the board
                        .add(new MvntAction(pose2d(Config.BACKSPOT, 27.5, 90))) // move to the board while moving the lift up
                        .add(new LiftAction(Config.LIFT_BACK, Config.liftMotorPowerAuton * 1.1), true)
                        .add(new MvntAction(Config.BOARDSPEED, 0, 0), 800)  // drive into the wall
                        .add(new MvntAction(Config.BOARDSPEED, 0, 0), 400)  // continue driving into the wall and move lift down(to tilt the pixel)
                        .add(new LiftAction(Config.LIFT_BACK - 100, Config.liftMotorPowerDown), true)
                        .add(new MvntAction(Config.BOARDSPEED, 0, 0), 200)  // continue driving and drop the yellow pixel
                        .add(new ClawAction(ClawAction.ClawStates.topOpen), true)
                        .add(new MvntAction(pose2d(24, 27, 90)))             // move back from the board and lower lift
                        .add(new LiftAction(Config.FLOOR + 40, Config.liftMotorPowerDown * 0.8), true)
                        .add(new MvntAction(pose2d(24, 3, 90)))              // move to the wall
                        .add(new MvntAction(pose2d(45, 3, 90)));             // park into the slot
            }
        }

        tel.addData("lift", 0);
        tel.addData("liftPower", 0);
        tel.addData("liftTarget", 0);
        tel.update();
        path.resetTimer();
        while (opModeIsActive()) {
            if (!path.run()) {
                robot.lift.setLiftPower(0);
                robot.drive.setDrivePowers(0, 0, 0, 0);
            }
            tel.addData("objectcenter: ", objectCenter[0]);
            tel.addData("direction", direction);
            tel.addData("paths remaining", path.actions.size());
            tel.addData("lift", robot.lift.encoder.getCurrentPosition());
            tel.addData("Pose", odometry.getPose().toString());
            tel.update();
        }
    }
    
    public static Pose2d pose2d(double x, double y, double heading) {
        return new Pose2d(x, y, new Rotation2d(Math.toRadians(heading)));
    }
}