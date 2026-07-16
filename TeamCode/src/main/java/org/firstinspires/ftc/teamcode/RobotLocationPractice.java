package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
public class RobotLocationPractice{
    double angle;
    double x;

    double y;
    // constructor method
    public RobotLocationPractice(double angle) {
        this.angle = angle;


    }

    public double getHeading(){
        //This method normalizes robot heading between -180 and 180 degrees
        //This is useful for calculating turn angles, especially when crossing the 0,360 boundary


        double angle = this.angle; //copy the angle on the imu
        while (angle > 180){
            angle -= 360; //subtract until in target range
        }

        while (angle <= -180){
            angle += 360; // add until in target range
        }
        return angle; // return the normalized value

    }

    public void turnRobot(double angleChange){
        angle += angleChange;
    }

    public void setAngle(double angle){

        this.angle = angle;
    }

    public double getAngle() {
        return this.angle;
    }

    public void changeX(double changeAmount) {
        x += changeAmount;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getX() {
        return this.x;
    }

    public void changeY(double changeY){
        y += changeY;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getY() {
        return this.y;
    }
}
