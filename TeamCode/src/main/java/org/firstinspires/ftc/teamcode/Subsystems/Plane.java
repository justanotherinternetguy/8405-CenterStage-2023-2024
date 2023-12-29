package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.teamcode.Auton.Config;

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
