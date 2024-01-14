package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.Config;

import java.util.function.Supplier;

public class Hang {
    public DcMotorEx hangMotor;
    public Gamepad gamepad;


    public Hang(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
    }


    public void input(Gamepad gamepad1, Supplier<Boolean> opMode, Supplier<Boolean> stop, ElapsedTime timer) {
        if (timer.seconds() < 2 * 60) return; // not endgame yet
        if (gamepad1.dpad_up) {
            setHangMotorPower(0.7);
        } else if (gamepad1.dpad_down) {
            hangNow(opMode, stop);
        }
        setHangMotorPower(0);
    }
    public void setHangMotorPower(double power) {
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setPower(power);
    }

    public void hangNow(Supplier<Boolean> opMode, Supplier<Boolean> stop) {
        while (opMode.get() && !stop.get()) {
            hangMotor.setPower(-Config.hangPower);
        }
    }
}
