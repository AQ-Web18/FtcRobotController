package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IfPractice extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        boolean a = gamepad1.a; // press TRUE, depress FALSE

        if (a) {

        telemetry.addData("A Botton","Pressed");

        }

        else {

        telemetry.addData("A Button", "Not Pressed");

        }

        telemetry.addData("A Botton State",a);



    }
}
