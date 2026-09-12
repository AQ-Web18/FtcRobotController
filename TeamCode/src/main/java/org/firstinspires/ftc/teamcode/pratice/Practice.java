package org.firstinspires.ftc.teamcode.pratice;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous // You can use Autonomous or TeleOp
public class Practice extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","Aqeel");
    }

    @Override
    public void loop() {

    }
}