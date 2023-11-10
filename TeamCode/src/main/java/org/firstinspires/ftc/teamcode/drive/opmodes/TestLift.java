package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Config;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="Lift Test", group="Linear Opmode")
public class TestLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Robot robot = new Robot(hardwareMap, gamepad1);
        robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.lift.rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Encoder leftEnc = new Encoder(robot.lift.leftLift);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        while(opModeIsActive())
        {
            if (gamepad1.left_trigger > 0.1) {
                robot.lift.leftLift.setPower(-gamepad1.left_trigger);
                robot.lift.rightLift.setPower(-gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1) {
                robot.lift.leftLift.setPower(gamepad1.right_trigger);
                robot.lift.rightLift.setPower(gamepad1.right_trigger);
            } else {
                robot.lift.leftLift.setPower(Config.gravity);
                robot.lift.rightLift.setPower(Config.gravity);
            }
            tel.addData("pos", leftEnc.getCurrentPosition());
            tel.addData("velo", leftEnc.getCorrectedVelocity());
            tel.addData("left", gamepad1.left_trigger);
            tel.addData("right", gamepad1.right_trigger);
            tel.update();
        }

    }
}
