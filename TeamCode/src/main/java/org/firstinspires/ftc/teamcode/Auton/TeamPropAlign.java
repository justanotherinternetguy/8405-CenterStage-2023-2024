package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Floor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AprilTags.AprilTagsInit;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Control.Rotate;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.ObjectDet.ObjectDetector;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class TeamPropAlign extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ObjectDetector teamPropDet;
        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        Odometry odometry = new Odometry(hardwareMap, robot.drive.imu);
//        AprilTagsInit init;
        PID.Config translationConfig = new PID.Config(Config.translationP, Config.translationI, Config.translationD);
        PID.Config rotationConfig = new PID.Config(Config.rotationP, Config.rotationI, Config.rotationD);
        Movement movement = new Movement(robot.drive, rrDrive, this::opModeIsActive, translationConfig, rotationConfig, Config.tolerance, telemetry);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        teamPropDet = new ObjectDetector(hardwareMap, tel);
//        robot.claw.setPower(Config.bottomServoClose, Config.topServoClose);
        waitForStart();
        odometry.reset();
        robot.drive.imu.resetYaw();

        Point[] paths = new Point[0];


        int[] objectCenter = teamPropDet.search();
        double third = 1920.0/3+100; // middle of camera, change later
        int direction = -1;
        if (objectCenter != null) {
            int centerX = objectCenter[0];
            if (centerX < third) { // left
                direction = 0;
                paths = new Point[]{
                        new Point(new Pose2d(0, 26, new Rotation2d(Math.toRadians(0))), new LiftPoint(Config.FLOOR)),
                        new Point(new Pose2d(-2.5, 26, new Rotation2d(Math.toRadians(-90))), false, true),
                        new Point(new Pose2d(24*1.25, 26, new Rotation2d(Math.toRadians(90))))
                };
            }
            else if (centerX > 2 * third) { // right
                direction = 1;
                paths = new Point[]{
                        new Point(new Pose2d(0, 26, new Rotation2d(Math.toRadians(0))), new LiftPoint(Config.FLOOR)),
                        new Point(new Pose2d(2.5, 26, new Rotation2d(Math.toRadians(90))), false, true),
                        new Point(new Pose2d(0, 24 * 0.5, new Rotation2d(Math.toRadians(90)))),
                        new Point(new Pose2d(24*1.25, 26, new Rotation2d(Math.toRadians(90))))
                };
            }
            else {
                direction = 2;
                paths = new Point[]{ // center
                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), new LiftPoint(Config.FLOOR, Config.liftMotorPowerAuton)),
                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(0))), true, true),
                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(0))), false, true),
                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(0))), new LiftPoint(200, Config.liftMotorPowerAuton)),
                        new Point(new Pose2d(24*1.25, 24, new Rotation2d(Math.toRadians(90))), new LiftPoint(400, Config.liftMotorPowerAuton)),
                        new Point(new Pose2d(24*1.5, 27, new Rotation2d(Math.toRadians(90))), new LiftPoint(Config.LIFT_BACK, Config.liftMotorPowerAuton)),
                        new Point(new Pose2d(24*1.55, 27, new Rotation2d(Math.toRadians(90))), new LiftPoint(Config.LIFT_BACK, Config.liftMotorPowerAuton), true, true),
                        new Point(new Pose2d(24*1.55, 27, new Rotation2d(Math.toRadians(90))), new LiftPoint(Config.LIFT_BACK, Config.liftMotorPowerAuton), false, false),
                };
            }
        }

        int pathOn = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            if (pathOn != paths.length) {
                if (paths[pathOn].move(robot, movement)) {
                    pathOn++;
                    timer = new ElapsedTime();
                    robot.drive.setDrivePowers(0, 0, 0, 0);
                }

            } else {
                robot.lift.liftToPos(Point.lastLiftHeight, Config.liftMotorPowerHold);
                robot.drive.setDrivePowers(0, 0, 0, 0);
            }
            tel.addData("objectcenter: ", objectCenter[0]);
            tel.addData("direction", direction);
            tel.addData("path on", pathOn);
            tel.addData("lift", robot.lift.leftLift.getCurrentPosition());
            tel.addData("holdPoint", Point.lastLiftHeight);
            tel.addData("Pose", odometry.getPose().toString());
            tel.update();
        }
    }

    public static class LiftPoint {
        public int height;
        public double power;
        public LiftPoint(int height, double power) {
            this.height = height;
            this.power = power;
        }
        public LiftPoint(int height) {
            this.height = height;
            this.power = Config.liftMotorPowerMacro;
        }
    }

    public static class Point {
        public Pose2d pose = null;
        public LiftPoint lift = null;
        public Boolean bottomClaw = null;
        public Boolean topClaw = null;
        public static int lastLiftHeight;
        public Point(Pose2d pose, LiftPoint lift, boolean bottom, boolean top) {
            this.pose = pose;
            this.lift = lift;
            this.bottomClaw = bottom;
            this.topClaw = top;
        }
        public Point(Pose2d pose) {
            this.pose = pose;
        }
        public Point(LiftPoint lift) {
            this.lift = lift;
        }
        public Point(boolean bottomClaw, boolean topClaw) {
            this.bottomClaw = bottomClaw;
            this.topClaw = topClaw;
        }
        public Point(Pose2d pose, LiftPoint lift) {
            this.pose = pose;
            this.lift = lift;
        }
        public Point(Pose2d pose, boolean bottomClaw, boolean topClaw) {
            this.pose = pose;
            this.bottomClaw = bottomClaw;
            this.topClaw = topClaw;
        }
        public Point(LiftPoint lift, boolean bottomClaw, boolean topClaw) {
            this.lift = lift;
            this.bottomClaw = bottomClaw;
            this.topClaw = topClaw;
        }
        public boolean move(Robot robot, Movement movement) {
            boolean atPose = true;
            if (this.pose != null) {
                atPose = !movement.move(this.pose);
            }
            Boolean atLift;
            if (this.lift != null) {
                lastLiftHeight = lift.height;
                atLift = Math.abs(robot.lift.liftToPos(lift.height, lift.power)) < 10;
            } else {
                atLift = Math.abs(robot.lift.liftToPos(lastLiftHeight, Config.liftMotorPowerHold)) < 10;
            }
            if (this.bottomClaw != null) {
                double bottomPower = this.bottomClaw ? Config.bottomServoClose : Config.bottomServoOpen;
                double topPower = this.topClaw ? Config.topServoClose : Config.topServoOpen;
                robot.claw.setPower(bottomPower, topPower);
            }
            return atPose && atLift;
        }
    }
}