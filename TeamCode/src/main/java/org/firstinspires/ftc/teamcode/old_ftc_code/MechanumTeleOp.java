package org.firstinspires.ftc.teamcode.old_ftc_code;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
@TeleOp(name = "Mechanum TeleOp")
public class MechanumTeleOp extends OpMode {

    // Drive motors (mecanum)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Mechanisms
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * Launcher velocities in encoder ticks per second (28 ticks per revolution).
     * The min velocity is the threshold used in launch() before the intake gets extra power.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    // Drive powers (kept as fields for telemetry)
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    // Intake power, shared between the triggers and launch()
    double intakePower;


    @Override
    public void init() {
        // Drive motors (names must match the robot configuration)
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back");

        // Mechanisms
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftintakeservo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightintakeservo");

        // Motor directions from the mecanum code
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake mode for a more controllable drivetrain
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Launcher velocity control
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        // Initialize servos
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Mecanum drive
        mecanumDrive();

        // Intake: right trigger in, left trigger out
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        // Launcher + windmill (may modify intakePower, so call before writing it)
        launch();

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        // Telemetry
        telemetry.addData("Front Left Power", "%.2f", frontLeftPower);
        telemetry.addData("Back Left Power", "%.2f", backLeftPower);
        telemetry.addData("Front Right Power", "%.2f", frontRightPower);
        telemetry.addData("Back Right Power", "%.2f", backRightPower);
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        telemetry.addLine();
    }

    @Override
    public void stop() {
    }
    void mecanumDrive() {
        // Stick mapping carried over from the original mecanum code
        double y = -gamepad1.right_stick_x; // Stick up is negative, invert to make forward positive
        double x = gamepad1.right_stick_y * 1.1; // Strafe multiplier to counter friction
        double rx = gamepad1.left_stick_x; // Turn

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    void launch() {
        // Right bumper spins up the launcher
        if (gamepad1.right_bumper) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        // Add intake power when the launcher is up to speed
        if (gamepad1.right_bumper && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            intakePower += 1;
        }

        // Left bumper runs the windmill feeder
        if (gamepad1.left_bumper) {
            windmillServo.setPower(1);
        } else {
            windmillServo.setPower(0);
        }
    }
}