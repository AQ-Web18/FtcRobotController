package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class Variable extends OpMode {
    @Override
    public void init() {
        int teamNum = 545858;
        double motorSpeed = .75;
        boolean clawClosed = true;
        String name = "RA Royals";
        int motorAngle = 90;

        telemetry.addData("Team Number", teamNum);
        telemetry.addData("Motor Speed",motorSpeed);
        telemetry.addData("Claw Closed", clawClosed);
        telemetry.addData("Name", name);
        telemetry.addData("Motor Angle", motorAngle);
    }

    @Override
    public void loop() {

    }
}

