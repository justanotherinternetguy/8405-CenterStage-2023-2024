package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import java.util.function.Supplier;

public class Movement {
    Drive drive;
    Odometry odom;
    PID driveXPID;
    PID driveYPID;
    Supplier<Boolean> opModeIsActive;
    double tolerance;
    Telemetry telemetry;

    public Movement(Drive drive, Odometry odom, Supplier<Boolean> opModeIsActive, double kP, double kI, double kD, double tolerance, Telemetry telemetry) {
        this.drive = drive;
        this.odom = odom;
        this.driveXPID = new PID(kP, kI, kD);
        this.driveYPID = new PID(kP, kI, kD);
        this.opModeIsActive = opModeIsActive;
        this.tolerance = tolerance;
        this.telemetry = telemetry;
    }

//    public void move(double x, double y) {
//
//        driveXPID.reset();
//        driveYPID.reset();
//        odom.update();
//
////        double targetX = 25;
////        double targetY = 25;
//        while ((Math.abs(x - odom.getX()) > this.tolerance || Math.abs(y - odom.getY()) > this.tolerance) && opModeIsActive.get()) {
//            odom.update();
//            double xError = x - odom.getX();
//            double yError = y - odom.getY();
//            double xPower = driveXPID.getValue(xError);
//            double yPower = driveYPID.getValue(yError);
//            drive.mecanumDrive(yPower, xPower, 0);
//            odom.update();
//        }
//    }

    public void move(double targetX, double targetY) {
        while (opModeIsActive.get() && (Math.abs(targetX - odom.getX()) > tolerance || Math.abs(targetY - odom.getY()) > tolerance)) {
            odom.update();
//            double x = driveXPID.getValue(targetX - odom.getX());
//            double y = driveYPID.getValue(targetY - odom.getY());
            double x = (targetX - odom.getX()) * 0.05;
            double y = (targetY - odom.getY()) * 0.05;
            double botHeading = Math.toRadians(odom.getHeading());
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX), 1);
            double frontLeftPower = (rotY + rotX) / denominator;
            double backLeftPower = (rotY - rotX) / denominator;
            double frontRightPower = (rotY - rotX) / denominator;
            double backRightPower = (rotY + rotX) / denominator;


            telemetry.addData("Pose ", odom.getPose().toString());
            telemetry.addData("x Error ", x);
            telemetry.addData("y Error ", y);
            telemetry.addData("botHeading", Math.toDegrees(botHeading));
            telemetry.addData("rotX  ", rotX);
            telemetry.addData("rotY  ", rotY);
            telemetry.addData("FL ", frontLeftPower);
            telemetry.addData("FR ", frontRightPower);
            telemetry.addData("BL ", backLeftPower);
            telemetry.addData("BR ", backRightPower);
            telemetry.update();

//            drive.setDrivePowers(frontLeftPower * 0.3, frontRightPower * 0.3, backLeftPower * 0.3, backRightPower * 0.3);
        }
        drive.setDrivePowers(0,0,0,0);
    }
}