package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Lift {
    private Gamepad gamepad;

    public enum LIFT_MODE {MANUAL, MACRO, HOLD, RESET, KILL, POWEROFF, NONE}

    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    public Encoder encoder;
    public LIFT_MODE currentMode;

    public int holdingPos;

    public enum LAST_KEY_PRESSED {NONE, A, B, X, Y}

    public LAST_KEY_PRESSED last_key_pressed = LAST_KEY_PRESSED.NONE;

    private PID pid;

    public ElapsedTime timer = new ElapsedTime();
    public boolean hasResetKill = false;
    public boolean startedKill = false;

    public Lift(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        currentMode = LIFT_MODE.NONE;
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        encoder = new Encoder(rightLift);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //encoder.setDirection(Encoder.Direction.REVERSE);

        pid = new PID(Config.liftP, Config.liftI, Config.liftD);

        holdingPos = -1;
    }

    public void liftTeleOp(Gamepad gamepad, Telemetry tel) {
        this.gamepad = gamepad;

        if (currentMode != LIFT_MODE.KILL) {
            if (gamepad.right_trigger > 0.2 || gamepad.left_trigger > 0.2) {
                currentMode = LIFT_MODE.MANUAL;
                holdingPos = -1;
                liftManual(gamepad, tel);
            } else if (gamepad.a || currentMode == LIFT_MODE.MACRO) {
                currentMode = LIFT_MODE.MACRO;
                holdingPos = -1;
                liftMacro(gamepad);
            } else if (gamepad.b || currentMode == LIFT_MODE.POWEROFF) {
                currentMode = LIFT_MODE.POWEROFF;
                holdingPos = -1;
                leftLift.setPower(0);
                rightLift.setPower(0);
                leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                setLiftPower(Config.gravity);
            }
        } else {
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
                if (Math.abs(encoder.getCurrentPosition() - Config.FLOOR) < 20) {
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
        double p = Range.clip(pid.getPower(target, encoder.getCurrentPosition()), -power, power);
        setLiftPower(p);
        return encoder.getCurrentPosition() - target;
    }


    public void liftManual(Gamepad gamepad, Telemetry tel) {
        if (gamepad.right_trigger > 0.2 && encoder.getCurrentPosition() < Config.LIFT_MAX) {
            setLiftPower(gamepad.right_trigger - Config.gravity); // triggers were flipped 4 some reason
            tel.addData("lp", -gamepad.right_trigger - Config.gravity);
        } else if (gamepad.left_trigger > 0.2) {
            setLiftPower(-gamepad.left_trigger);
            tel.addData("lp", gamepad.left_trigger);
        }
        tel.addData("left", leftLift.getCurrentPosition());
        tel.addData("right", rightLift.getCurrentPosition());
        tel.addData("enc", encoder.getCurrentPosition());

    }

    private void liftMacro(Gamepad gamepad) {

        if (gamepad.a) {
            last_key_pressed = LAST_KEY_PRESSED.A;
            liftToBase();
        }
//        else if(gamepad.b)
        else if (last_key_pressed == LAST_KEY_PRESSED.A) {
            liftToBase();
        }
    }

    public void setLiftPower(double power) {
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setPower(power * Config.liftMotorPowerMultTeleOp * -1);
        rightLift.setPower(power * Config.liftMotorPowerMultTeleOp * -1);
    }

    public void liftToBase() {
        currentMode = LIFT_MODE.MACRO;
        liftToPos(400, (Config.liftMotorPowerMacro));// * 1.1) + Config.gravity);
    }
}
