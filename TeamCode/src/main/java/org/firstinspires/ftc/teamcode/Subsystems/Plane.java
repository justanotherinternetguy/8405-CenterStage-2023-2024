package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane {
    public Servo droneServo;
    public Gamepad gamepad;

    public Plane(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        droneServo = hardwareMap.get(Servo.class, "droneServo");
    }

    public void input(Gamepad gamepad1) {
        if (gamepad1.dpad_up) {
            droneServo.setPosition(0.2);
        }
    }
}
