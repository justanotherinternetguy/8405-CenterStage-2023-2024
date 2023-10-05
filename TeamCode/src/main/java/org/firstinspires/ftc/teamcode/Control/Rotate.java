package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Controllers.PID;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.utils;

public class Rotate {
    private Drive drive;
    private Odometry odom;
    private Telemetry telemetry;
    private Supplier<Boolean> opModeIsActive;
    private PID rotatePID;
    private double tolerance;

    public Rotate(Drive drive, Odometry odom, Telemetry telemetry, Supplier<Boolean> opModeIsActive, PID rotatePID, double tolerance) {
        this.drive = drive;
        this.odom = odom;
        this.telemetry = telemetry;
        this.opModeIsActive = opModeIsActive;
        this.rotatePID = rotatePID;
        this.tolerance = tolerance;
    }

    public void rotate(double targetHeading) {
        odom.update();
        rotatePID.reset();
        double error;
        odom.update();
        while (Math.abs(utils.angleDifference(targetHeading, odom.getHeading())) > tolerance && opModeIsActive.get()) {
            odom.update();
            error = utils.angleDifference(targetHeading, odom.getHeading());
            double headingPower = rotatePID.getValue(error);
            drive.mecanumDrive(0, 0, headingPower);
            telemetry.addData("heading ", odom.getHeading());
            telemetry.addData("error ", error);
            telemetry.addData("headingPower ", headingPower);
            telemetry.update();
        }
    }
}