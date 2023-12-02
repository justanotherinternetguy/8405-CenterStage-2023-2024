package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.teamcode.Auton.Config;

public class Claw {
    public Servo clawServo;
    public Servo bottomServo;
    public Servo topServo;

    public Gamepad gamepad;

    public boolean bottomClaw = false; // is closed?
    public boolean topClaw = false; // is closed?
    public boolean isBackboard = false; // is angled?
    public boolean lastLeftBumper = false;
    public boolean lastRightBumper = false;
    public boolean lastBurstButtom = false;
    public ElapsedTime timer = new ElapsedTime();
    public boolean inBurst = false;

    public Claw(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        topServo = hardwareMap.get(Servo.class, "frontServo");
        bottomServo = hardwareMap.get(Servo.class, "backServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    public void setPos(double bPos, double tPos) {
        bottomServo.setPosition(bPos);
        topServo.setPosition(tPos);
    }

    public void input(Gamepad gamepad1) {
        //true = closed
//        if (gamepad1.left_bumper && !lastLeftBumper) {
//            topClaw = !topClaw;
//        }
        if (gamepad1.right_bumper && !lastRightBumper) {
            isBackboard = !isBackboard;
        }
        if (gamepad1.y && !lastBurstButtom) {
            if (topClaw == true && bottomClaw == true) {
                bottomClaw = !bottomClaw;
                timer.reset();
                inBurst = true;
            } else if (topClaw == false && bottomClaw == false) {
                topClaw = !topClaw;
                bottomClaw = !bottomClaw;
            }
        }
        if (inBurst && timer.milliseconds() > Config.burstDelay && topClaw == true && bottomClaw == false) {
            topClaw = !topClaw;
            inBurst = false;
        }
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;
        lastBurstButtom = gamepad1.y;
        if (bottomClaw) {
            bottomServo.setPosition(Config.bottomServoClose);
        } else {
            bottomServo.setPosition(Config.bottomServoOpen);
        }
        if (topClaw) {
            topServo.setPosition(Config.topServoClose);
        } else {
            topServo.setPosition(Config.topServoOpen);
        }
        if (isBackboard) {
            clawServo.setPosition(Config.clawServoBackboard);
        } else {
            clawServo.setPosition(Config.clawServoFloor);
        }
    }
}
