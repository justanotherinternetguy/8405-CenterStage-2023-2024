package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drive {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;


    public Drive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE); // motor direction

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void mecanumDrive(double power, double strafe, double turn) {
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (power + strafe + turn) / denominator;
        double backLeftPower = (power - strafe + turn) / denominator;
        double frontRightPower = (power - strafe - turn) / denominator;
        double backRightPower = (power + strafe - turn) / denominator;

        setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void setDrivePowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}
