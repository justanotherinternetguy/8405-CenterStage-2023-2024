package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name="Claw Test", group="Linear Opmode")
public class TestLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Robot robot = new Robot(hardwareMap, gamepad1);
        while(opModeIsActive())
        {
            if(gamepad1.right_trigger > 0.1)
            {
                robot.claw.setPower(Config.bottomServoClose, Config.topServoClose);
            }
            else if(gamepad1.left_trigger > 0.1)
            {
                robot.claw.setPower(Config.bottomServoClose, Config.topServoClose);
            }
            telemetry.addData("Power Bottom: ", robot.claw.bottomServo.getPower());
            telemetry.addData("Power Top: ", robot.claw.topServo.getPower());
            telemetry.update();

        }

    }
}
