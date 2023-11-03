package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public CRServo servo1;
    public CRServo servo2;
    public Gamepad gamepad;

    public Claw(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
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
            servo1.setPower(0);
            servo2.setPower(0);
        }
    }

    private void intake() {
        servo1.setPower(0.2);
        servo2.setPower(-0.2);
    }
    private void outtake() {
        servo1.setPower(-0.2);
        servo2.setPower(0.2);
    }
}
