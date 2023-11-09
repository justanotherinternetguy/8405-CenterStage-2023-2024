package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Lift {
    private Gamepad gamepad;

    public enum LIFT_MODE {MANUAL, MACRO, HOLD, RESET, KILL, NONE}
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    public LIFT_MODE currentMode;

    public int holdingPos;

    public enum LAST_KEY_PRESSED{NONE, A, B, X, Y}

    public LAST_KEY_PRESSED last_key_pressed = LAST_KEY_PRESSED.NONE;

    private double P = 0.05 ;

    public ElapsedTime timer = new ElapsedTime();
    public boolean hasResetKill = false;
    public boolean startedKill = false;

    public Lift(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        currentMode = LIFT_MODE.NONE;
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        holdingPos = -1;
    }
    public void liftTeleOp(Gamepad gamepad) {
        this.gamepad = gamepad;

        if (currentMode != LIFT_MODE.KILL) {
            if (gamepad.right_trigger > 0.2 || gamepad.left_trigger > 0.2) {
                currentMode = LIFT_MODE.MANUAL;
                holdingPos = -1;
                liftManual(gamepad);
            }
            else if(gamepad.a || gamepad.b || currentMode == LIFT_MODE.MACRO) {
                currentMode = LIFT_MODE.MACRO;
                holdingPos = -1;
                liftMacro(gamepad);
            }
            else
            {
                currentMode = LIFT_MODE.HOLD;
                if(holdingPos == -1)
                {
                    holdingPos = Math.min(leftLift.getCurrentPosition(), Config.LIFT_MAX);
                }
                liftToPos(holdingPos,  Config.liftMotorPowerHold);
            }

        }
        else {
            if (!startedKill) {
                timer.reset();
                startedKill = true;
            }
            if (timer.milliseconds() >= Config.KILLTIME) {
                if (!hasResetKill) {
//                    setLiftPower(0);
                    rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hasResetKill = true;
                }
                liftToPos(Config.FLOOR, Config.liftMotorPowerMacro);
                if (Math.abs(leftLift.getCurrentPosition() - Config.FLOOR) < 20) {
                    currentMode = LIFT_MODE.NONE;
                    hasResetKill = false;
                    startedKill = false;
                }
            } else {
                hasResetKill = false;
                setLiftPower(-Config.liftMotorPowerMacro);
            }
        }
    }

    public double liftToPos(int target, double power) {

        double multiplier = (target - leftLift.getCurrentPosition()) * P;
        if(Math.abs(multiplier) > 1)
        {
            multiplier = multiplier/Math.abs(multiplier);//sets to either -1 or 1
        }
        rightLift.setPower(-power*multiplier);//max power at which the lift can run at
        leftLift.setPower(-power*multiplier);//max power at which the lift can run at

        return leftLift.getCurrentPosition() - target;
    }

    public void liftManual(Gamepad gamepad) {
        if (gamepad.right_trigger > 0.3 && leftLift.getCurrentPosition() < Config.LIFT_MAX) {
            setLiftPower(gamepad.right_trigger);
        }
        else if (gamepad.left_trigger > 0.3) {
            setLiftPower(-gamepad.left_trigger);
        }
//        else
//        {
//            setLiftPower(0);
//            currentMode = LIFT_MODE.NONE;
//        }

    }

    private void liftMacro(Gamepad gamepad) {

        if (gamepad.a) {
            last_key_pressed = LAST_KEY_PRESSED.A;
            liftToBase();
        }
        else if(gamepad.b)
        {
            last_key_pressed = LAST_KEY_PRESSED.B;
            this.currentMode = LIFT_MODE.KILL;
        }
        else if(last_key_pressed == LAST_KEY_PRESSED.A)
        {
            liftToBase();
        }
    }

    public void setLiftPower(double power) {
        leftLift.setPower(-power * Config.liftMotorPowerMultTeleOp);
        rightLift.setPower(-power * Config.liftMotorPowerMultTeleOp);
    }

    public void liftToBase() {
        currentMode = LIFT_MODE.MACRO;
        liftToPos(200, Config.liftMotorPowerMacro);
    }
}
