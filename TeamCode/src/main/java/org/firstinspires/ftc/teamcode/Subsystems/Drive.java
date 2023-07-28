package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drive {
    public DcMotorEx testMotor; // TEST

    public Drive(HardwareMap hardwareMap) {
        testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
    }

    public void moveTest(double power) {
        testMotor.setPower(power);
    }
}
