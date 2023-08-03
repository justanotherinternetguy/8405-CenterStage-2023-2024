package org.firstinspires.ftc.teamcode.Control;

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

    public Movement(Drive drive, Odometry odom, Supplier<Boolean> opModeIsActive, double kP, double kI, double kD, double tolerance) {
        this.drive = drive;
        this.odom = odom;
        this.driveXPID = new PID(kP, kI, kD);
        this.driveYPID = new PID(kP, kI, kD);
        this.opModeIsActive = opModeIsActive;
        this.tolerance = tolerance;
    }

    public void move(double x, double y, double tolerance) {

        driveXPID.reset();
        driveYPID.reset();
        odom.update();

        double targetX = 25;
        double targetY = 25;
        while ((Math.abs(targetX - odom.x) > tolerance || Math.abs(targetY - odom.y) > tolerance) && opModeIsActive.get()) {
            odom.update();
            double xError = targetX - odom.x;
            double yError = targetY - odom.y;
            double xPower = driveXPID.getValue(xError);
            double yPower = driveYPID.getValue(yError);
            drive.mecanumDrive(yPower, xPower, 0);
            odom.update();
        }
    }
}
