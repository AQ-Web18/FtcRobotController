package org.firstinspires.ftc.teamcode.ftc_code;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.Distance_Sensor;

@TeleOp(name = "Tank Mode", group = "TeleOp")
public class TankMode extends OpMode {

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;

    // Variables for drive motor power.
    double leftPower;
    double rightPower;

    // Variable for intake power.
    double intakePower;

    // Distance sensor
    Distance_Sensor ds = new Distance_Sensor();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize hardware.
        leftDrive = hardwareMap.get(DcMotor.class, "leftdrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightdrive");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftintakeservo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightintakeservo");

        /*
         * Set motor directions.
         *
         * These settings assume the same drivetrain orientation
         * as your original StarterBot code.
         */
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Brake when joystick is released.
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Initialize intake servos.
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

        // Reverse the right intake servo so both sides pull inward.
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize distance sensor.
        ds.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after INIT but before START.
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when START is pressed.
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after START.
     */
    @Override
    public void loop() {

        /*
         * TANK DRIVE
         *
         * Left joystick controls the left side.
         * Right joystick controls the right side.
         *
         * The Y-axis is reversed because pushing a joystick
         * forward produces a negative value.
         */
        leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;

        // Send power to the drive motors.
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        /*
         * INTAKE
         *
         * Right trigger = intake
         * Left trigger = reverse intake
         */
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        /*
         * TELEMETRY
         */
        telemetry.addData(
                "Motors",
                "left (%.2f), right (%.2f)",
                leftPower,
                rightPower
        );

        telemetry.addData(
                "Triggers",
                "left (%.2f), right (%.2f)",
                gamepad1.left_trigger,
                gamepad1.right_trigger
        );

        telemetry.addData("Distance"+" (in)",ds.getDistance());

        telemetry.update();
    }

    /*
     * Code to run ONCE after STOP.
     */
    @Override
    public void stop() {
    }
}