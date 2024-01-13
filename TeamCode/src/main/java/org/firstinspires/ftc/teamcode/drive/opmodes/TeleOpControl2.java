package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectDet.ObjectDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Motor Drive", group = "Linear Opmode")
public class TeleOpControl2 extends LinearOpMode {
    public static boolean slowMode = false;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, gamepad1);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime timer = new ElapsedTime();

//        AprilTagsInit apriltags = new AprilTagsInit(hardwareMap, telemetry);
//        apriltags.initialize(telemetry);

//        TwoWheelTrackingLocalizer tw = new TwoWheelTrackingLocalizer(hardwareMap, drive);

        Telemetry tel = FtcDashboard.getInstance().getTelemetry();


        ObjectDetector objectDetector = new ObjectDetector(hardwareMap, tel);

        boolean lastX = false;

        waitForStart();
        robot.drive.imu.resetYaw();
        timer.reset();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                robot.drive.setDrivePowers(1, 0, 0, 0);
            }
            if (gamepad1.b) {
                robot.drive.setDrivePowers(0, 1, 0, 0);
            }
            if (gamepad1.a) {
                robot.drive.setDrivePowers(0, 0, 0, 1);
            }
            if (gamepad1.x) {
                robot.drive.setDrivePowers(0, 0, 1, 0);
            }
        }
    }
}