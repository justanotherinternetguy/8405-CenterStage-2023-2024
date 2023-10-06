package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public IMU imu;

    public Drive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE); // motor direction

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backRight.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);
    }

    public void mecanumDrive(double power, double strafe, double turn) {
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (power + strafe + turn) / denominator;
        double backLeftPower = (power - strafe + turn) / denominator;
        double frontRightPower = (power - strafe - turn) / denominator;
        double backRightPower = (power + strafe - turn) / denominator;

        setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void setDrivePowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    // TEMP
    public double getIMU() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
