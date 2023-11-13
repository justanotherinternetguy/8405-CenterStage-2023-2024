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
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AUTON_BLUE_NEAR extends LinearOpMode {
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

        Point[] paths = new Point[0];


        int[] objectCenter = teamPropDet.search();
        double third = 1920.0/3+100; // middle of camera, change later
        int direction = -1;
        if (objectCenter != null) {
            int centerX = objectCenter[0];
            if (centerX < third) { // left
                direction = 0;
                paths = new Point[]{
                        new Point(Config.powerMultiplier * 0.8),
                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), new LiftPoint(Config.FLOOR, Config.liftMotorFloor + Config.gravity), true, true),
                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true, true),
                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(0)))),
                        new Point(new Pose2d(-3, 30, new Rotation2d(Math.toRadians(-90))), false, true),
                        new Point(new Pose2d(-3, 30, new Rotation2d(Math.toRadians(-90))), new LiftPoint(100, Config.liftMotorPowerAuton + Config.gravity),false, true),
//                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(90))), new LiftPoint(300, Config.liftMotorPowerAuton + Config.gravity)),
                        new Point(new Pose2d(1, 30, new Rotation2d(Math.toRadians(-90)))),
                        new Point(new Pose2d(1, 14, new Rotation2d(Math.toRadians(-90)))),
                        new Point(new Pose2d(-12, 14, new Rotation2d(Math.toRadians(-90)))),
                        new Point(Config.powerMultiplier * 0.9),
//                        new Point(new Pose2d(24, 22, new Rotation2d(Math.toRadians(90))), new LiftPoint(500, Config.liftMotorPowerAuton + Config.gravity)),
                        new Point(new Pose2d(-24, 22, new Rotation2d(Math.toRadians(-90)))),
                        new Point(new Pose2d(-24, 22, new Rotation2d(Math.toRadians(-90))), new LiftPoint(Config.LIFT_BACK, (Config.liftMotorPowerAuton * 1.1) + Config.gravity)),
                        new Point(Config.BOARDSPEED * 1.4),
                        new Point(new Pose2d(-Config.BACKSPOT, 23, new Rotation2d(Math.toRadians(-90)))),
                        new Point(new double[]{Config.BOARDSPEED, 0.0, 0.0}, 2500.0),
//                        new Point(new LiftPoint(Config.LIFT_BACK - 75, Config.liftMotorPowerDown)),
                        new Point(new LiftPoint(Config.LIFT_BACK - 100, Config.liftMotorPowerDown * 0.75)),
                        new Point(false, false),
                        new Point(Config.powerMultiplier),
                        new Point(new Pose2d(-24, 22, new Rotation2d(Math.toRadians(-90))), new LiftPoint(Config.FLOOR+30, Config.liftMotorPowerDown * 0.8)),
                        new Point(new Pose2d(-24, 3, new Rotation2d(Math.toRadians(-90)))),
                        new Point(new Pose2d(-45, 3, new Rotation2d(Math.toRadians(-90)))),
                };
            }
            else if (centerX > 2 * third) { // right
                direction = 1;
                Config.FLOOR = 27;
                paths = new Point[]{
//                        new Point(Config.powerMultiplier * 0.8),
//                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), new LiftPoint(Config.FLOOR, Config.liftMotorFloor + Config.gravity)),
//                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true, true),
//                        new Point(new Pose2d(0, 30, new Rotation2d(Math.toRadians(0)))),
//                        new Point(new Pose2d(3, 30, new Rotation2d(Math.toRadians(90))), new LiftPoint(Config.FLOOR+35, Config.liftMotorFloor + Config.gravity), false, true),
//                        new Point(new Pose2d(3, 30, new Rotation2d(Math.toRadians(90))), new LiftPoint(100, (Config.liftMotorPowerAuton * 1.1) + Config.gravity), false, true),
////                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(90))), new LiftPoint(300, Config.liftMotorPowerAuton + Config.gravity)),
//                        new Point(new Pose2d(-2, 28, new Rotation2d(Math.toRadians(90)))),
//                        new Point(new Pose2d(-2, 28, new Rotation2d(Math.toRadians(-90)))),
//                        new Point(Config.powerMultiplier * 0.8),
//                        new Point(new Pose2d(-24, 33, new Rotation2d(Math.toRadians(-90))), new LiftPoint(500, Config.liftMotorPowerAuton + Config.gravity)),
//                        new Point(new Pose2d(-24, 33, new Rotation2d(Math.toRadians(-90))), new LiftPoint(Config.LIFT_BACK, (Config.liftMotorPowerAuton * 1.1) + Config.gravity)),
//                        new Point(Config.BOARDSPEED * 1.15),
//                        new Point(new Pose2d(-Config.BACKSPOT, 33, new Rotation2d(Math.toRadians(-90)))),
//                        new Point(new double[]{Config.BOARDSPEED, 0.0, 0.0}, 2500.0),
////                        new Point(new LiftPoint(Config.LIFT_BACK - 75, Config.liftMotorPowerDown)),
//                        new Point(new LiftPoint(Config.LIFT_BACK - 100, Config.liftMotorPowerDown * 0.8)),
//                        new Point(false, false),
//                        new Point(Config.powerMultiplier),
//                        new Point(new Pose2d(-20, 34.5, new Rotation2d(Math.toRadians(-90))), new LiftPoint(Config.FLOOR + 100, Config.liftMotorPowerDown * 0.6)),
//                        new Point(new Pose2d(-20, 1, new Rotation2d(Math.toRadians(-90)))),
//                        new Point(new Pose2d(-45, 1, new Rotation2d(Math.toRadians(-90)))),

                                                new Point(Config.powerMultiplier * 0.8),
                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), new LiftPoint(Config.FLOOR, Config.liftMotorFloor + Config.gravity), true, true),
                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true, true),
                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(0)))),
                        new Point(new Pose2d(3, 30, new Rotation2d(Math.toRadians(90))), false, true),
                        new Point(new Pose2d(3, 30, new Rotation2d(Math.toRadians(90))), new LiftPoint(100, Config.liftMotorPowerAuton + Config.gravity),false, true),
//                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(90))), new LiftPoint(300, Config.liftMotorPowerAuton + Config.gravity)),
                        new Point(new Pose2d(-1, 30, new Rotation2d(Math.toRadians(90)))),
                        new Point(new Pose2d(-1, 14, new Rotation2d(Math.toRadians(90)))),
                        new Point(new Pose2d(-12, 14, new Rotation2d(Math.toRadians(-90)))),
                        new Point(Config.powerMultiplier * 0.9),
//                        new Point(new Pose2d(24, 22, new Rotation2d(Math.toRadians(90))), new LiftPoint(500, Config.liftMotorPowerAuton + Config.gravity)),
                        new Point(new Pose2d(-24, 22, new Rotation2d(Math.toRadians(-90)))),
                        new Point(new Pose2d(-24, 22, new Rotation2d(Math.toRadians(-90))), new LiftPoint(Config.LIFT_BACK, (Config.liftMotorPowerAuton * 1.1) + Config.gravity)),
                        new Point(Config.BOARDSPEED * 1.4),
                        new Point(new Pose2d(-Config.BACKSPOT, 23, new Rotation2d(Math.toRadians(-90)))),
                        new Point(new double[]{Config.BOARDSPEED, 0.0, 0.0}, 2500.0),
//                        new Point(new LiftPoint(Config.LIFT_BACK - 75, Config.liftMotorPowerDown)),
                        new Point(new LiftPoint(Config.LIFT_BACK - 100, Config.liftMotorPowerDown * 0.8)),
                        new Point(false, false),
                        new Point(Config.powerMultiplier),
                        new Point(new Pose2d(-24, 22, new Rotation2d(Math.toRadians(-90))), new LiftPoint(Config.FLOOR+30, Config.liftMotorPowerDown * 0.8)),
                        new Point(new Pose2d(-24, 3, new Rotation2d(Math.toRadians(-90)))),
                        new Point(new Pose2d(-45, 3, new Rotation2d(Math.toRadians(-90)))),
                };
            }
            else {
                direction = 2;
                paths = new Point[]{ // center
                        new Point(Config.powerMultiplier*0.9),
                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), new LiftPoint(Config.FLOOR, Config.liftMotorFloor + Config.gravity)),
                        new Point(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true, true),
                        new Point(new Pose2d(0, 28.5, new Rotation2d(Math.toRadians(0))), false, true),
                        new Point(new Pose2d(0, 28, new Rotation2d(Math.toRadians(0))), new LiftPoint(300, Config.liftMotorPowerAuton + Config.gravity)),
                        new Point(new Pose2d(0, 26, new Rotation2d(Math.toRadians(0)))),
                        new Point(new Pose2d(-24, 27, new Rotation2d(Math.toRadians(-90))), new LiftPoint(500, Config.liftMotorPowerAuton + Config.gravity)),
                        new Point(Config.powerMultiplier * 1.15),
                        new Point(new Pose2d(-24, 27, new Rotation2d(Math.toRadians(-90)))),
                        new Point(Config.BOARDSPEED*0.9),
                        new Point(new Pose2d(-Config.BACKSPOT, 28, new Rotation2d(Math.toRadians(-90))), new LiftPoint(Config.LIFT_BACK, (Config.liftMotorPowerAuton * 1.25) + Config.gravity)),
                        new Point(new double[]{Config.BOARDSPEED, 0.0, 0.0}, 1700.0),
                        new Point(false, false),
//                        new Point(new LiftPoint(Config.LIFT_BACK - 75, Config.liftMotorPowerDown)),
                        new Point(new Pose2d(-33, 28, new Rotation2d(Math.toRadians(-90))), new LiftPoint(Config.LIFT_BACK - 350, Config.liftMotorPowerDown * 0.7)),
                        new Point(false, false),
                        new Point(Config.powerMultiplier),
                        new Point(new Pose2d(-24, 27, new Rotation2d(Math.toRadians(-90))), new LiftPoint(Config.FLOOR + 50, Config.liftMotorPowerDown * 0.9)),
                        new Point(new Pose2d(-24, 3, new Rotation2d(Math.toRadians(-90)))),
                        new Point(new Pose2d(-45, 3, new Rotation2d(Math.toRadians(-90)))),
                };
            }
        }

        int pathOn = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        tel.addData("lift", 0);
        tel.addData("liftPower", 0);
        tel.addData("liftTarget", 0);
        tel.update();
        while (opModeIsActive()) {
            if (pathOn != paths.length) {
                if (paths[pathOn].move(robot, movement, tel)) {
                    pathOn++;
                    robot.drive.setDrivePowers(0, 0, 0, 0);
                    timer.reset();
                } else if (timer.milliseconds() > 2500) {
                    pathOn++;
                    robot.drive.setDrivePowers(0, 0, 0, 0);
                    timer.reset();
                }

            } else {
                robot.lift.liftToPos(Point.lastLiftHeight, Config.liftMotorPowerHold);
                robot.drive.setDrivePowers(0, 0, 0, 0);
            }
            tel.addData("objectcenter: ", objectCenter[0]);
            tel.addData("direction", direction);
            tel.addData("path on", pathOn);
            tel.addData("lift", robot.lift.encoder.getCurrentPosition());
            tel.addData("holdPoint", Point.lastLiftHeight);
            tel.addData("Pose", odometry.getPose().toString());
            tel.update();
            telemetry.addData("dir", direction);
            telemetry.update();
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

    public static class CAFPid {
        /*
         * Proportional Integral Derivative Controller w/ Low pass filter and anti-windup
         * Credit: CtrlAltFtc
         */

        public double Kp;
        public double Ki;
        public double Kd;

        public double lastTarget = 0;
        public double integralSum = 0;

        public double lastError = 0;

        public double maxIntegralSum;

        public double a = 0.8; // a can be anything from 0 < a < 1
        public double previousFilterEstimate = 0;
        public double currentFilterEstimate = 0;

        ElapsedTime timer = new ElapsedTime();

        public CAFPid(PID.Config pid) {
            this.Kp = pid.getKp();
            this.Ki = pid.getKi();
            this.Kd = pid.getKd();
            this.maxIntegralSum = 0.25/this.Ki;
        }

        public double calc(double target, double current) {
                // calculate the error
                double error = target - current;

                double errorChange = (error - lastError);

                // filter out hight frequency noise to increase derivative performance
                currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
                previousFilterEstimate = currentFilterEstimate;

                // rate of change of the error
                double derivative = currentFilterEstimate / timer.seconds();

                // sum of all error over time
                integralSum = integralSum + (error * timer.seconds());


                // max out integral sum
                if (integralSum > maxIntegralSum) {
                    integralSum = maxIntegralSum;
                }

                if (integralSum < -maxIntegralSum) {
                    integralSum = -maxIntegralSum;
                }

                // reset integral sum upon setpoint changes
                if (target != lastTarget) {
                    integralSum = 0;
                }

                double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                lastError = error;

                lastTarget = target;

                // reset the timer for next time
                timer.reset();
                return out;
            }
    }

    public static class Point {
        public Pose2d pose = null;
        public LiftPoint lift = null;
        public Boolean bottomClaw = null;
        public Boolean topClaw = null;
        public static int lastLiftHeight;
        public static double drivePower = Config.powerMultiplier;
        public Double speed = null;
        public CAFPid liftPID = new CAFPid(new PID.Config(Config.liftP, Config.liftI, Config.liftD));
        public double[] powers;
        public Double duration = null;
        public ElapsedTime timer = null;
        public Point(double speed) {
            this.speed = speed;
        }
        public Point(double[] powers, Double duration) {
            this.powers = powers;
            this.duration = duration;
        }
        public Point(double[] powers, Double duration, boolean bottom, boolean top) {
            this.powers = powers;
            this.duration = duration;
            this.bottomClaw = bottom;
            this.topClaw = top;
        }
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
        public boolean move(Robot robot, Movement movement, Telemetry tel) {
            if (this.duration != null) {
                if (this.timer == null) {
                    timer = new ElapsedTime();
                }
                if (timer.milliseconds() > this.duration) {
                    robot.drive.setDrivePowers(0,0,0,0);
                    return true;
                } else {
                    robot.drive.mecanumDrive(this.powers[0], this.powers[1], this.powers[2]);
                    return false;
                }
            }

            timer = null;

            if (this.speed != null) {
                drivePower = this.speed;
            }

            boolean atPose = true;
            if (this.pose != null) {
                atPose = !movement.move(this.pose, drivePower);
            }
            boolean atLift = true;
            if (this.lift != null) {
                lastLiftHeight = lift.height;
                atLift = Math.abs(robot.lift.liftToPos(lift.height, lift.power)) < Config.liftTolerance;
                double power = Range.clip(liftPID.calc(lift.height, robot.lift.encoder.getCurrentPosition()) * lift.power, -lift.power, lift.power);
                tel.addData("liftPower", power);
                tel.addData("liftTarget", lift.height);
                robot.lift.leftLift.setPower(power);
                robot.lift.rightLift.setPower(power);
//                robot.lift.leftLift.setPower(lift.power);
//                robot.lift.rightLift.setPower(lift.power);
//                robot.lift.leftLift.setTargetPosition(-lift.height);
//                robot.lift.rightLift.setTargetPosition(lift.height);
//                robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return Math.abs(robot.lift.encoder.getCurrentPosition() - lift.height) < Config.liftTolerance;
            } else {
                robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.leftLift.setPower(Config.gravity);
                robot.lift.rightLift.setPower(Config.gravity);
                tel.addData("liftPower", Config.gravity);
                tel.addData("liftTarget", lastLiftHeight);
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