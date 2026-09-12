package org.firstinspires.ftc.teamcode.add_onns;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Servos;

@Disabled
@TeleOp
public class ServoExamples extends OpMode {
    Servos s = new Servos();

    @Override
    public void init() {
        s.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            s.setServoPos(1);
        }
        else {
            s.setServoPos(0);
        }

        if (gamepad1.b){
            s.setServoCr(1);
        }
        else {
            s.setServoCr(0);
        }
    }
}
