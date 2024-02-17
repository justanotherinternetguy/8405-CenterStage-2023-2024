package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;

public class Claw {
    public Servo clawServo;
    public Servo bottomServo;
    public Servo topServo;

    public Gamepad gamepad;

    public boolean bottomClaw = false; // is closed?
    public boolean topClaw = false; // is closed?
    public boolean isBackboard = false; // is angled?
    public boolean lastB = false;
    public boolean lastY = false;
    public boolean lastRB = false;
    public ElapsedTime timer = new ElapsedTime();

    public Claw(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        topServo = hardwareMap.get(Servo.class, "frontServo");
        bottomServo = hardwareMap.get(Servo.class, "backServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    public void input(Gamepad gamepad1, Telemetry tel) {
        tel.addData("topClaw", topClaw);
        tel.addData("bottomClaw", bottomClaw);

        if (gamepad1.y && !lastY) {
            if (bottomClaw) {
                bottomClaw = false; // if bottom is closed, open it
            } else {
                bottomClaw = true; // if bottom is open then close both
                topClaw = true;
            }
        }

        if (gamepad1.b && !lastB) {
            if (!bottomClaw) {
                topClaw = false; // if bottom claw is open, open top
            }
        }

        //true = closed
        if (gamepad1.right_bumper && !lastRB) {
            isBackboard = !isBackboard;
        }
        if (topClaw) {
            topServo.setPosition(Config.topServoClose);
        } else {
            topServo.setPosition(Config.topServoOpen);
        }
        if (bottomClaw) {
            bottomServo.setPosition(Config.bottomServoClose);
        } else {
            bottomServo.setPosition(Config.bottomServoOpen);
        }
        if (isBackboard) {
//            clawServo.getController().pwmEnable();
            clawServo.setPosition(Config.clawServoBackboard);
        } else {
            clawServo.setPosition(Config.clawServoFloor);
//            clawServo.getController().pwmDisable();
        }

        lastB = gamepad1.b;
        lastY = gamepad1.y;
        lastRB = gamepad1.right_bumper;
    }
}
