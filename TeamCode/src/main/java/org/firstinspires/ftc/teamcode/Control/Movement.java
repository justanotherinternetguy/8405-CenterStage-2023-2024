package org.firstinspires.ftc.teamcode.Control;

import android.app.admin.DeviceAdminService;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils;

import java.util.Arrays;
import java.util.function.Supplier;

public class Movement {
    private Drive drive;
    private SampleMecanumDrive rrDrive;
    private PID driveXPID;
    private PID driveYPID;
    private PID headingPID;
    private Supplier<Boolean> opModeIsActive;
    private double tolerance;
    private double toleranceH;
    private Telemetry telemetry;
    public enum DIRECTION{LEFT, RIGHT};
    private ElapsedTime timer;

    public Movement(Drive drive, SampleMecanumDrive rrDrive, Supplier<Boolean> opModeIsActive, PID.Config drivePIDConfig, PID.Config headingPIDConfig, double tolerance, double toleranceH, Telemetry telemetry) {
        this.drive = drive;
        this.rrDrive = rrDrive;
        this.driveXPID = new PID(drivePIDConfig);
        this.driveYPID = new PID(drivePIDConfig);
        this.headingPID = new PID(headingPIDConfig);
        this.opModeIsActive = opModeIsActive;
        this.tolerance = tolerance;
        this.toleranceH = toleranceH;
        this.telemetry = telemetry;

        timer = new ElapsedTime();
    }

    // we can probably turn the code in teleOp, strafeAt and move for absolute movement into a function that
    // accepts x,y, rx, and heading(we can also add power but we can also just lower x, y, heading like how it works with pid)

    public void strafeAt(double power, Pose2d initial, DIRECTION direction)//attempts to perfectly strafe either left or right
    {
        Pose2d pose = rrDrive.getPose();
        double y = driveYPID.getValue(initial.getY()-pose.getY());
        double rx = headingPID.getValue(utils.angleDifference(Math.toDegrees(initial.getHeading()), Math.toDegrees(pose.getHeading())));
        if(direction == DIRECTION.LEFT)
        {
            power = -power;//reverse power to tell it to go left
        }
        double botHeading = -pose.getRotation().getRadians();
        double rotX = power * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = power * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        drive.setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public boolean move(Pose2d target) {
        return this.move(target, Config.powerMultiplier);
    }

    public boolean move(Pose2d target, double power) {
        Pose2d init = rrDrive.getPose();
        Pose2d init_target_pose = new Pose2d(target.getX() - init.getX(), target.getY() - init.getY(), new Rotation2d(Math.toRadians(utils.angleDifference(target.getRotation().getDegrees(), init.getRotation().getDegrees()))));
        timer.reset();
        double elapsed_time;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashTelem = dashboard.getTelemetry();

        //while (opModeIsActive.get() && (Math.abs(target.getX() - pose.getX()) > tolerance || Math.abs(target.getY() - pose.getY()) > tolerance || Math.abs(utils.angleDifference(target.getRotation().getDegrees(), pose.getRotation().getDegrees())) > tolerance * 3)) {
        elapsed_time = timer.seconds();
        rrDrive.update();
        Pose2d pose = rrDrive.getPose();

        //this didn't work cause we never updated elapsed time in the loop - be my guest if you still wanna try
            // elapsed_time is updated at the very top of the loop, we didn't really seem to have much issues with the actual accel, decell but rather random
            // movements(such as the bot would drive out to like 4,12 then to 0,24 rather than directly to 24
            // that and trying to match pid and profiles enough(i think we can try using profile as a cap rather than a value for the pid)
        //but elapsed time should then be a function parameter: NO LOOPS IN HERE PLEASE
//        double instantTargetPositionX = MotionProfile.motion_profile(Config.MAX_ACCEL, Config.MAX_VELOCITY, init_target_pose.getX(), elapsed_time) + init.getX();
//        double instantTargetPositionY = MotionProfile.motion_profile(Config.MAX_ACCEL, Config.MAX_VELOCITY, init_target_pose.getY(), elapsed_time) + init.getY(); // (-90 - 90) + 90 = -180 + 90 = -90
//        double instantTargetPositionH = MotionProfile.motion_profile(Config.MAX_ACCEL, Config.MAX_VELOCITY, init_target_pose.getRotation().getDegrees(), elapsed_time)  + Math.toDegrees(init.getHeading());

        double x = driveXPID.getValue(target.getX() - pose.getX());
        double y = driveYPID.getValue(target.getY() - pose.getY());
        double rx = headingPID.getValue(utils.angleDifference(target.getRotation().getDegrees(), Math.toDegrees(pose.getHeading())));
        double botHeading = -pose.getRotation().getRadians();    //i won't change it cause it seems to work but y?
                                                                    // just smt that happened when trying to get two wheel working, fixed in getPose()
        double[] powers = absMovement(x, y, rx, botHeading);

        //dont spam telemetry on the actual driver hub, use dash if u want all this data

        dashTelem.addData("power", Arrays.toString(new Double[] {x, y, rx}));
        dashTelem.addData("error Heading", utils.angleDifference(target.getRotation().getDegrees(), Math.toDegrees(pose.getHeading())));
        dashTelem.addData("error X", target.getX() - pose.getX());
        dashTelem.addData("error Y", target.getY() - pose.getY());
        dashTelem.addData("target", target);
        dashTelem.addData("pose", pose);
        dashTelem.update();


        drive.setDrivePowers(powers[0]*power, powers[1]*power, powers[2]*power, powers[3]*power);
        //returns if we're there for the outside loop. can easily change to &&'s(which I recommend)

        return Math.abs(target.getX() - pose.getX()) > tolerance || Math.abs(target.getY() - pose.getY()) > tolerance || Math.abs(utils.angleDifference(target.getRotation().getDegrees(), pose.getRotation().getDegrees())) > toleranceH;
    }

    public static double[] absMovement(double x, double y, double rx, double botHeading) {
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * Config.XMULTI;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        return new double[] { frontLeftPower, frontRightPower, backLeftPower, backRightPower };
    }
}