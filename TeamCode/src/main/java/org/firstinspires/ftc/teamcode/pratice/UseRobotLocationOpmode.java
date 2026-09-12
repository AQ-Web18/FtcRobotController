package org.firstinspires.ftc.teamcode.pratice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class UseRobotLocationOpmode extends OpMode {
    RobotLocationPractice rlp= new RobotLocationPractice(0);

    @Override
    public void init() {
        rlp.setAngle(0);
        rlp.setX(0);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            rlp.turnRobot(0.1);
        }
        else if (gamepad1.b){
            rlp.turnRobot(-0.1);
        }

        if (gamepad1.dpad_left){
            rlp.changeX(0.1);
        } else if (gamepad1.dpad_right) {
            rlp.changeX(-0.1);
        }

        if (gamepad1.dpad_up){
            rlp.changeY(0.1);
        }
        else if (gamepad1.dpad_down) {
            rlp.changeY(-0.1);
        }

        telemetry.addData("Heading",rlp.getHeading());
        telemetry.addData("Angle", rlp.getAngle());
        telemetry.addData("X Value",rlp.getX());
        telemetry.addData("Y Value", rlp.getY());
    }
}
