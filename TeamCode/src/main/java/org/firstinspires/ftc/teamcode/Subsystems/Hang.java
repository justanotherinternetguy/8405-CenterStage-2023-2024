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

    public ElapsedTime timer = new ElapsedTime();
    public int lastInput = 0;//0 for none, 1 for dpadup, 2 for dpad2

    public Gamepad gamepad;


    public Hang(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;

        hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
    }

    public void input(Gamepad gamepad1, Supplier<Boolean> opMode, Supplier<Boolean> stop, ElapsedTime timer) {
        if (timer.seconds() < 2 * 60) return; // not endgame yet
        if (gamepad1.dpad_up) {
            this.timer.reset();
            setHangMotorPower(0.7);
        } else if (gamepad1.dpad_down) {
            timer.reset();
            hangNow(opMode, stop);
        }
        else if(timer.milliseconds() > 50)
        {
            setHangMotorPower(0);
        }
    }
    public void setHangMotorPower(double power) {
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setPower(power);
    }

    public void hangNow(Supplier<Boolean> opMode, Supplier<Boolean> stop) {
        while (opMode.get() && !stop.get()) {
            hangMotor.setPower(Config.hangPower);
        }
    }

}
