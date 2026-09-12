package org.firstinspires.ftc.teamcode.pratice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class GamePad extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        // runs 50 times a sec

        double correctleft = -gamepad1.left_stick_y;

        telemetry.addData("Left Joystick x",gamepad1.left_stick_x);
        telemetry.addData("Left Joystick y",correctleft);
        telemetry.addData("a",gamepad1.a);

        double correctright = -gamepad1.right_stick_y;

        telemetry.addData("Right Joystick x",gamepad1.right_stick_x);
        telemetry.addData("Right Joystick y",correctright);
        telemetry.addData("b", gamepad1.b);

        double difference = gamepad1.left_stick_x - gamepad1.right_stick_x;

        telemetry.addData("Difference",difference);

        double sum = gamepad1.left_trigger + gamepad1.right_trigger;

        telemetry.addData("Sum",sum);
    }
}


