package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AprilTags.AprilTagsInit;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name="Test TeleOP", group="Linear Opmode")
public class TestTeleOP extends OpMode {
    AprilTagsInit apriltags;
    int apriltag_x = 960;
    int apriltag_y = 400;
    Robot robot;

    double[] lastPos = null;
    @Override
    public void init()
    {
        apriltags = new AprilTagsInit(hardwareMap, telemetry);
        apriltags.initialize(telemetry);

        robot = new Robot(hardwareMap, gamepad1);
    }
    @Override
    public void init_loop()
    {

    }
    @Override
    public void start()
    {

    }
    @Override
    public void loop() {
        int tag = 1;
        double thres = 10;
        double[] pos = apriltags.searchFor(tag);
        if(pos == null && lastPos != null)
        {
            pos = lastPos;
        }
        if(pos != null) {
            telemetry.addData("X: ", pos[1]);
            if (pos[1] - apriltag_x < -thres)
            {
                robot.drive.mecanumDrive(0, -0.3, 0);

            }
            else if(pos[1] - apriltag_x > thres)
            {
                robot.drive.mecanumDrive(0, 0.3, 0);
            }
            else
            {
                telemetry.addLine("DONE!");
            }
            lastPos = pos;
        }
    }
}
