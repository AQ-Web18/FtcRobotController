package org.firstinspires.ftc.teamcode.add_onns;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.DC_Motors;

@Disabled
@TeleOp
public class DCMotorPractice extends OpMode {
    DC_Motors dcm = new DC_Motors();

    @Override
    public void init() {
        dcm.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            dcm.setMotorSpeed(1);
        }
        else {
            dcm.setMotorSpeed(0);
        }

        if (gamepad1.b){
            dcm.setMotorSpeed(-1);
        }
        else {
            dcm.setMotorSpeed(0);
        }
        telemetry.addData("Motor Revs",dcm.getTicksPerRev());
    }
}
