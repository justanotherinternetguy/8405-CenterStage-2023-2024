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
                path.add(new MvntAction(new Pose2d(0, 28, new Rotation2d(0))))                          // drive to tape
                        .add(new MvntAction(new Pose2d(-3, 28, new Rotation2d(Math.toRadians(-90)))))           // face the tape
                        .add(new ClawAction(ClawAction.ClawStates.bottomOpen))                                         // drop the purple pixel
                        .add(new MvntAction(new Pose2d(0, 28, new Rotation2d(Math.toRadians(-90)))))            // move back and up the claw
                        .add(new LiftAction(300, Config.liftMotorPowerAuton), true)
                        .add(new MvntAction(new Pose2d(0, 28, new Rotation2d(Math.toRadians(90)))))             // face the board
                        .add(new MvntAction(new Pose2d(Config.BACKSPOT, 34.5, new Rotation2d(Math.toRadians(90))))) // move to the board and raise lifts
                        .add(new LiftAction(Config.LIFT_BACK, Config.liftMotorPowerAuton * 1.1), true)
                        .add(new MvntAction(new double[] {Config.BOARDSPEED, 0, 0}), 800)                     // drive into the wall
                        .add(new MvntAction(new double[] {Config.BOARDSPEED, 0, 0}), 400)                     // continue driving into the wall and move lift down(to tilt the pixel)
                        .add(new LiftAction(Config.LIFT_BACK - 100, Config.liftMotorPowerDown), true)
                        .add(new MvntAction(new double[] {Config.BOARDSPEED, 0, 0}), 200)                     // continue driving and drop the yellow pixel
                        .add(new ClawAction(ClawAction.ClawStates.topOpen), true)
                        .add(new MvntAction(new Pose2d(24, 34.5, new Rotation2d(Math.toRadians(90)))))          // move back from the board and lower lift
                        .add(new LiftAction(Config.FLOOR + 40, Config.liftMotorPowerDown * 0.8), true)
                        .add(new MvntAction(new Pose2d(24, 3, new Rotation2d(90))))                     // move to the wall
                        .add(new MvntAction(new Pose2d(45, 3, new Rotation2d(90))));
//                };
            }
            else if (centerX > 2 * third) { // right
                direction = 1;
//                paths = new Point[]{
//                        new Point(Config.powerMultiplier * 0.8),
//                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), new LiftPoint(Config.FLOOR, Config.liftMotorFloor + Config.gravity), true, true),
//                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true, true),
//                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(0)))),
//                        new Point(new Pose2d(3, 30, new Rotation2d(Math.toRadians(90))), false, true),
//                        new Point(new Pose2d(3, 30, new Rotation2d(Math.toRadians(90))), new LiftPoint(100, Config.liftMotorPowerAuton + Config.gravity),false, true),
////                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(90))), new LiftPoint(300, Config.liftMotorPowerAuton + Config.gravity)),
//                        new Point(new Pose2d(-1, 30, new Rotation2d(Math.toRadians(90)))),
//                        new Point(new Pose2d(-1, 14, new Rotation2d(Math.toRadians(90)))),
//                        new Point(new Pose2d(12, 14, new Rotation2d(Math.toRadians(90)))),
//                        new Point(Config.powerMultiplier * 0.9),
////                        new Point(new Pose2d(24, 22, new Rotation2d(Math.toRadians(90))), new LiftPoint(500, Config.liftMotorPowerAuton + Config.gravity)),
//                        new Point(new Pose2d(24, 22, new Rotation2d(Math.toRadians(90)))),
//                        new Point(new Pose2d(24, 22, new Rotation2d(Math.toRadians(90))), new LiftPoint(Config.LIFT_BACK, (Config.liftMotorPowerAuton * 1.1) + Config.gravity)),
//                        new Point(Config.BOARDSPEED * 1.4),
//                        new Point(new Pose2d(Config.BACKSPOT, 24, new Rotation2d(Math.toRadians(90)))),
//                        new Point(new double[]{Config.BOARDSPEED, 0.0, 0.0}, 1900.0),
////                        new Point(new LiftPoint(Config.LIFT_BACK - 75, Config.liftMotorPowerDown)),
//                        new Point(new LiftPoint(Config.LIFT_BACK - 100, Config.liftMotorPowerDown * 0.8)),
//                        new Point(false, false),
//                        new Point(Config.powerMultiplier),
//                        new Point(new Pose2d(24, 22, new Rotation2d(Math.toRadians(90))), new LiftPoint(Config.FLOOR+30, Config.liftMotorPowerDown * 0.8)),
//                        new Point(new Pose2d(24, 3, new Rotation2d(Math.toRadians(90)))),
//                        new Point(new Pose2d(45, 3, new Rotation2d(Math.toRadians(90)))),
//                };
            }
            else {
                direction = 2;
                path.add(new MvntAction(new Pose2d(0, 28, new Rotation2d(0))))                            // move to tape
                        .add(new ClawAction(ClawAction.ClawStates.bottomOpen))                                           // drop purple pixel
                        .add(new LiftAction(300, Config.liftMotorPowerAuton))                                   // lift up height a bit while also moving back(to not knock over the purple pixel)
                        .add(new MvntAction(new Pose2d(0, 26, new Rotation2d(0))), true)
                        .add(new MvntAction(new Pose2d(0, 27.5, new Rotation2d(Math.toRadians(90)))))            // rotate towards the board
                        .add(new MvntAction(new Pose2d(Config.BACKSPOT, 27.5, new Rotation2d(Math.toRadians(90)))))  // move to the board while moving the lift up
                        .add(new LiftAction(Config.LIFT_BACK, Config.liftMotorPowerAuton * 1.1), true)
                        .add(new MvntAction(new double[] {Config.BOARDSPEED, 0, 0}), 800)                      // drive into the wall
                        .add(new MvntAction(new double[] {Config.BOARDSPEED, 0, 0}), 400)                      // continue driving into the wall and move lift down(to tilt the pixel)
                        .add(new LiftAction(Config.LIFT_BACK - 100, Config.liftMotorPowerDown), true)
                        .add(new MvntAction(new double[] {Config.BOARDSPEED, 0, 0}), 200)                      // continue driving and drop the yellow pixel
                        .add(new ClawAction(ClawAction.ClawStates.topOpen), true)
                        .add(new MvntAction(new Pose2d(24, 27, new Rotation2d(Math.toRadians(90)))))             // move back from the board and lower lift
                        .add(new LiftAction(Config.FLOOR + 40, Config.liftMotorPowerDown * 0.8), true)
                        .add(new MvntAction(new Pose2d(24, 3, new Rotation2d(90))))                       // move to the wall
                        .add(new MvntAction(new Pose2d(45, 3, new Rotation2d(90))));                      // park into the slot
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
}