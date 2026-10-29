package org.firstinspires.ftc.teamcode.old_ftc_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
/*
 * Drive Motor Test - spins ONE wheel at a time so you can check that:
 *   1. Each config name is plugged into the correct port (the right wheel spins)
 *   2. Each wheel spins FORWARD (the direction that would push the robot forward)
 *
 * Put the robot on a block so the wheels are off the ground.
 *
 * Hold a button to spin one wheel at 30% power:
 *   X (square)   -> left_front
 *   Y (triangle) -> right_front
 *   A (cross)    -> left_back
 *   B (circle)   -> right_back
 *
 * The directions below MUST match the ones in BioBuzzStarterbotTeleopMecanum.
 * If you change one here, change it there too.
 */
@TeleOp(name = "Drive Motor Test", group = "Test")
public class DriveMotorTest extends OpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private static final double TEST_POWER = 0.3;

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");

        // Same directions as the teleop
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized - robot on a block, wheels off the ground");
    }

    @Override
    public void loop() {
        leftFront.setPower(gamepad1.x ? TEST_POWER : 0);
        rightFront.setPower(gamepad1.y ? TEST_POWER : 0);
        leftBack.setPower(gamepad1.a ? TEST_POWER : 0);
        rightBack.setPower(gamepad1.b ? TEST_POWER : 0);

        telemetry.addLine("Hold X / Y / A / B to spin one wheel. Each should spin FORWARD.");
        telemetry.addData("X  left_front ", gamepad1.x ? "RUNNING" : "-");
        telemetry.addData("Y  right_front", gamepad1.y ? "RUNNING" : "-");
        telemetry.addData("A  left_back  ", gamepad1.a ? "RUNNING" : "-");
        telemetry.addData("B  right_back ", gamepad1.b ? "RUNNING" : "-");
    }
}