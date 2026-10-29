package org.firstinspires.ftc.teamcode.ftc_code;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Simple time-based autonomous for the mecanum StarterBot:
 *   1. Drive to a launch spot
 *   2. Spin up the launcher and feed with the windmill
 *   3. Drive/strafe off the line to park
 *
 * All distances are time-based, so TUNE the values in the "TUNING" section on your field.
 */
@Autonomous(name = "Mechanum Auto", group = "Auto")
public class MechanumAuto extends LinearOpMode {

    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo, rightIntakeServo, windmillServo;

    // ---------- TUNING ----------
    static final double DRIVE_POWER = 0.5;

    static final double TO_LAUNCH_SECONDS = 1.0;    // drive forward to launch spot
    static final double LAUNCH_VELOCITY = 1250;      // ticks/sec (matches TeleOp's actual launch speed)
    static final double SPINUP_TIMEOUT = 4.0;       // max seconds to wait for launcher speed
    static final double FEED_SECONDS = 6.0;         // how long to run the windmill/intake feeding
    static final double PARK_STRAFE_SECONDS = 1.0;  // strafe to park
    static final double STRAFE_DIRECTION = 1.0;     // 1.0 or -1.0 to flip strafe side (red/blue)
    // ----------------------------

    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftintakeservo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightintakeservo");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized - ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 1. Drive to launch spot
        drive(DRIVE_POWER, 0, 0, TO_LAUNCH_SECONDS);

        // 2. Spin up launcher, wait until it is close to speed (or timeout)
        launcher.setVelocity(LAUNCH_VELOCITY);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SPINUP_TIMEOUT
                && launcher.getVelocity() < LAUNCH_VELOCITY - 25) {
            telemetry.addData("Launcher", launcher.getVelocity());
            telemetry.update();
            idle();
        }

        // Feed with windmill + intake
        windmillServo.setPower(1);
        setIntake(1);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < FEED_SECONDS) {
            telemetry.addData("Launcher", launcher.getVelocity());
            telemetry.addData("Status", "Feeding");
            telemetry.update();
            idle();
        }
        windmillServo.setPower(0);
        setIntake(0);
        launcher.setVelocity(0);

        // 3. Park
        drive(0, STRAFE_DIRECTION * DRIVE_POWER, 0, PARK_STRAFE_SECONDS);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }

    private void setIntake(double power) {
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    /** Time-based mecanum move. y = forward, x = strafe, rx = turn (same mix as TeleOp). */
    private void drive(double y, double x, double rx, double seconds) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        leftFrontDrive.setPower((y + x + rx) / denominator);
        leftBackDrive.setPower((y - x + rx) / denominator);
        rightFrontDrive.setPower((y - x - rx) / denominator);
        rightBackDrive.setPower((y + x - rx) / denominator);

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            idle();
        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}