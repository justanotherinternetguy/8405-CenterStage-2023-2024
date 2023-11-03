package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drive drive;
    public Lift lift;
    public Claw claw;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        drive = new Drive(hardwareMap);
        lift = new Lift(hardwareMap, gamepad);
        claw = new Claw(hardwareMap, gamepad);
    }
}
