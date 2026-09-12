package org.firstinspires.ftc.teamcode.add_onns;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Distance_Sensor;
@Disabled
@TeleOp
public class DistanceSensorPractice extends OpMode {
    Distance_Sensor ds = new Distance_Sensor();

    @Override
    public void init() {
        ds.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Distance",ds.getDistance());

    }
}
