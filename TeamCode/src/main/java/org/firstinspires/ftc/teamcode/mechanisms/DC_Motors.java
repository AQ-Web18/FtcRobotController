package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class DC_Motors {
    private DcMotor motor;
    private double ticksPerRev; // Revolutions

    public void init(HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class,"motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRev = motor.getMotorType().getTicksPerRev();
    }

    public void setMotorSpeed(double speed) {
        //accepts values from -1.0 to 1.0

        motor.setPower(speed);
    }

    public double getTicksPerRev() {
        return motor.getCurrentPosition() / ticksPerRev; // normalizing ticks to revolutions
    }
}
