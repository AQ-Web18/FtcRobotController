package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    private Servo servoPos; //pos= positional servo
    private CRServo servoCr; //cr= continous rotation


    public void init(HardwareMap hwMap){
        servoPos = hwMap.get(Servo.class,"Positional Servo");
        servoCr = hwMap.get(CRServo.class,"Continous Rotational Servo");
        //servoPos.setDirection(Servo.Direction.REVERSE);
        //servoPos.scaleRange(.5,1);
        //servoCr.getDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setServoPos(double angle){
        servoPos.setPosition(angle);
    }

    public void setServoCr(double power) {
        servoCr.setPower(power);
    }


}
