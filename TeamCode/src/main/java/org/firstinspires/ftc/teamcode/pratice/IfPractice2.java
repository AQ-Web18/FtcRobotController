package org.firstinspires.ftc.teamcode.pratice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class IfPractice2 extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {

        double lefty = gamepad1.left_stick_y;

        if (lefty < 0) {
            telemetry.addData("Left Stick", "is Negative");
        }
        else if (lefty > .5) {
            telemetry.addData("Left Stick", "is greater than 50%");
        }
        else if (lefty > 0) {
            telemetry.addData("Left Stick", "Is greater than 0");
        }
        else {
            telemetry.addData("Left Stick","is Zero");
        }

    }
}

/*

AND - && if (LeftY < 8.5 && LeftY > 0) {

|| if (leftY < B || rightY < 0) {

NOT ! if (!clawClosed) {  OR

*/
