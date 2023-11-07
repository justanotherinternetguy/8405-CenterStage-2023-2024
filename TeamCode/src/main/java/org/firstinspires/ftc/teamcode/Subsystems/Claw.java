package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.teamcode.Auton.Config;

public class Claw {
    public CRServo bottomServo;
    public CRServo topServo;
    public CRServo frontServo;
    public CRServo backServo;
    public Gamepad gamepad;

    public boolean bottomClaw = false; // is closed?
    public boolean topClaw = false; // is closed?
    public boolean lastLeftBumper = false;
    public boolean lastRightBumper = false;

    public Claw(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        topServo = hardwareMap.get(CRServo.class, "frontservo");
        bottomServo = hardwareMap.get(CRServo.class, "backservo");
    }
    public void setPower(double bPower, double tPower)
    {
        bottomServo.setPower(bPower);
        topServo.setPower(tPower);
    }

    public void input(Gamepad gamepad1) {
        if (gamepad1.left_bumper && !lastLeftBumper) {
            topClaw = !topClaw;
        }
        if (gamepad1.right_bumper && !lastRightBumper) {
            bottomClaw = !bottomClaw;
        }
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
    }
}
