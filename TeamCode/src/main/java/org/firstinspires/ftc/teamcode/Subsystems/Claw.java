package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.teamcode.Auton.Config;

public class Claw {
<<<<<<< HEAD
    public CRServo bottomServo;
    public CRServo topServo;
=======
    public CRServo frontServo;
    public CRServo backServo;
>>>>>>> 03ecc7ad511930f7b0f4a6b214f55224422b2285
    public Gamepad gamepad;

    public boolean bottomClaw = false; // is closed?
    public boolean topClaw = false; // is closed?
    public boolean lastLeftBumper = false;
    public boolean lastRightBumper = false;

    public Claw(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
<<<<<<< HEAD
        topServo = hardwareMap.get(CRServo.class, "frontservo");
        bottomServo = hardwareMap.get(CRServo.class, "backservo");
    }
    public void setPower(double bPower, double tPower)
    {
        bottomServo.setPower(bPower);
        topServo.setPower(tPower);
=======
        frontServo = hardwareMap.get(CRServo.class, "frontServo");
        backServo = hardwareMap.get(CRServo.class, "backServo");
>>>>>>> 03ecc7ad511930f7b0f4a6b214f55224422b2285
    }

    public void input(Gamepad gamepad1) {
        if (gamepad1.left_bumper && !lastLeftBumper) {
            topClaw = !topClaw;
        }
        if (gamepad1.right_bumper && !lastRightBumper) {
            bottomClaw = !bottomClaw;
        }
<<<<<<< HEAD
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;
        if (bottomClaw) {
            bottomServo.setPower(Config.bottomServoClose);
        } else {
            bottomServo.setPower(Config.bottomServoOpen);
        }
        if (topClaw) {
            topServo.setPower(Config.topServoClose);
        } else {
            topServo.setPower(Config.topServoOpen);
        }
=======
        else {
            frontServo.setPower(0);
            backServo.setPower(0);
        }
    }

    private void intake() {
        frontServo.setPower(0.2);
        backServo.setPower(-0.2);
    }
    private void outtake() {
        frontServo.setPower(-0.2);
        backServo.setPower(0.2);
>>>>>>> 03ecc7ad511930f7b0f4a6b214f55224422b2285
    }
}
