package org.firstinspires.ftc.teamcode.add_onns;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class WebcamPractice extends OpMode {
    Webcam atw = new Webcam();

    @Override
    public void init() {
        atw.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        atw.update();
        AprilTagDetection id20 = atw.getTagBySpecificId(20);
        telemetry.addData("id20 String", id20.toString());
    }
}
