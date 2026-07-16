package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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