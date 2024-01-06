package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.teamcode.Auton.Config;

public class Hang {
    public DcMotorEx hangMotor;
    public Gamepad gamepad;


    public Hang(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
    }


    public void input(Gamepad gamepad1) {
        if (gamepad1.dpad_up) {
            setHangMotorPower(0.7);
        } else if (gamepad1.dpad_down) {
            hangNow();
        }
        setHangMotorPower(0);
    }
    public void setHangMotorPower(double power) {
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setPower(power);
    }

    public void hangNow() {
        for (;;) {
            hangMotor.setPower(Config.hangPower);
        }
    }
}
