package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Lift {
    private Gamepad gamepad;

    public enum LIFT_MODE {MANUAL, MACRO, HOLD, RESET, KILL, NONE}
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    public Motor.Encoder rightLiftEnc;
    public LIFT_MODE currentMode;

    public Lift(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        currentMode = LIFT_MODE.NONE;
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightLiftEnc = new MotorEx(hardwareMap, "rightLift").encoder;
        rightLiftEnc.setDirection(Motor.Direction.REVERSE);
    }
    public void liftTeleOp(Gamepad gamepad) {
        this.gamepad = gamepad;
        currentMode = LIFT_MODE.MANUAL;
        if (gamepad.right_trigger > 0.3 && rightLiftEnc.getPosition() < Config.LIFT_MAX) {
            rightLift.setPower(gamepad.right_trigger * Config.liftMotorPowerMult);
        }
        else if (gamepad.left_trigger > 0.3 && rightLiftEnc.getPosition() < Config.LIFT_MAX) {
            rightLift.setPower(gamepad.left_trigger * Config.liftMotorPowerMult * -1);
        }
        else { rightLift.setPower(0); }
    }
}
