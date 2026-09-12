package org.firstinspires.ftc.teamcode.add_onns;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Touch_Sensor;
@Disabled
@TeleOp
public class TouchSensorPractice extends OpMode {
    Touch_Sensor ts = new Touch_Sensor();

    @Override
    public void init() {
        ts.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (ts.getTouchSensorState()){
            telemetry.addData("Touch Sensor","Presses!");
        }
        else {
            telemetry.addData("Touch Sensor","Not Pressed!");
        }

    }
}
