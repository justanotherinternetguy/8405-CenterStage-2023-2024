package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Plane {
    public Servo droneServo;
    public Gamepad gamepad;
    Telemetry tel;

    public Plane(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        droneServo = hardwareMap.get(Servo.class, "droneServo");
    }

    public void input(Gamepad gamepad1) {
        if (gamepad1.left_bumper) {
            droneServo.setPosition(0);
        }
    }
}
