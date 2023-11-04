package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public CRServo frontServo;
    public CRServo backServo;
    public Gamepad gamepad;

    public Claw(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        frontServo = hardwareMap.get(CRServo.class, "frontServo");
        backServo = hardwareMap.get(CRServo.class, "backServo");
    }

    public void clawTeleOp(Gamepad gamepad) {
        this.gamepad = gamepad;
        if (gamepad.left_bumper) {
            intake();
        }
        if (gamepad.right_bumper) {
            outtake();
        }
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
    }
}
