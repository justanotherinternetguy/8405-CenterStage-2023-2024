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
    public Motor.Encoder leftLiftEnc;
    public LIFT_MODE currentMode;

    public int holdingPosLeft;
    public int holdingPosRight;

    public Lift(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        currentMode = LIFT_MODE.NONE;
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
<<<<<<< HEAD
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
=======
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
>>>>>>> b107fd9a20a194b9c4983ea46de673f5eb57141c

        rightLiftEnc = new MotorEx(hardwareMap, "rightLift").encoder;
        rightLiftEnc.setDirection(Motor.Direction.REVERSE);

        leftLiftEnc = new MotorEx(hardwareMap, "leftLift").encoder;
        holdingPosRight = -1;
        holdingPosLeft = -1;
    }
    public void liftTeleOp(Gamepad gamepad) {
        this.gamepad = gamepad;
        if (currentMode != LIFT_MODE.KILL) {
            if (gamepad.right_trigger > 0.2 || gamepad.left_trigger > 0.2) {
                currentMode = LIFT_MODE.MANUAL;
                liftManual(gamepad);
            }
            if(gamepad.square || gamepad.circle || gamepad.left_bumper || gamepad.triangle || gamepad.right_bumper) {
                currentMode = LIFT_MODE.MACRO;
                liftMacro(gamepad);
            }
        }
        else if (currentMode == LIFT_MODE.KILL) {
            if (rightLift.getCurrentPosition() > 100) {
                setLiftPower(-1);
            }
        }
    }

    public void liftToPos(int targetR, int targetL, double power) {
        if (Math.abs(targetR - rightLift.getCurrentPosition()) > 15) {
            rightLift.setTargetPosition(targetR);
            leftLift.setTargetPosition(targetL);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(power);
            leftLift.setPower(power);
        }
    }

    public void liftManual(Gamepad gamepad) {
        this.gamepad = gamepad;
        currentMode = LIFT_MODE.MANUAL;
        if (gamepad.right_trigger > 0.3 && rightLiftEnc.getPosition() < Config.LIFT_MAX) {
            setLiftPower(gamepad.right_trigger);
        }
        else if (gamepad.left_trigger > 0.3 && rightLiftEnc.getPosition() < Config.LIFT_MAX) {
            setLiftPower(gamepad.left_trigger * -1);
        }
        else { setLiftPower(0); }
    }

    private void liftMacro(Gamepad gamepad) {
        if (gamepad.square) {
            liftToBase();
        }
    }

    private void setLiftPower(double power) {
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setPower(power * Config.liftMotorPowerMultTeleOp);
        rightLift.setPower(power * Config.liftMotorPowerMultTeleOp);
    }

    public void liftToBase() {
        currentMode = LIFT_MODE.MACRO;
        liftToPos(200, 200, Config.liftMotorPowerMacro);
    }
}
