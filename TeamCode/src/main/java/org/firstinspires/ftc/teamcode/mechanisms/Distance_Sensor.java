package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance_Sensor {
    private DistanceSensor distance;

    public void init(HardwareMap hwmap){
        distance = hwmap.get(DistanceSensor.class,"distance_sensor");
    }

    public double getDistance() {
        return distance.getDistance(DistanceUnit.INCH);
    }
}
