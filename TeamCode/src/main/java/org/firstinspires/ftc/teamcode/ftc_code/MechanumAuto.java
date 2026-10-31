package org.firstinspires.ftc.teamcode.ftc_code;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Royal Robotics Guild (FTC 34324) autonomous for the goBILDA StarterBot with mecanum wheels.
 * Built from MechanumTeleOp: same hardware names, motor directions, shooter PIDF and
 * "ready to fire" check (shooter at 96% of target speed before the windmill feeds).
 *
 * SEQUENCE
 *   1. Drive forward to the shooting spot
 *   2. Spin up the shooter
 *   3. Feed elements (windmill + intake) only while the shooter is at speed
 *   4. Strafe sideways to park
 *
 * This is TIME BASED, so distances change with battery voltage. Tune the constants below
 * on a fully charged battery.
 */
@Autonomous(name = "Mechanum Auto", group = "StarterBot")
//@Disabled
public class MechanumAuto extends LinearOpMode {

    // ----- Tunables -----
    static final double DRIVE_POWER = 0.5;

    // Seconds to drive forward to the shooting spot (use a negative number to back up)
    static final double TO_SHOOT_SEC = 0.8;

    // Shooter (ticks per second, same units as the TeleOp)
    static final double SHOOTER_VELOCITY       = 1250;
    static final double SHOOTER_READY_FRACTION = 0.96;
    static final double SPINUP_TIMEOUT_SEC     = 3.0;  // fire anyway if it never reaches speed
    static final double SHOOT_DURATION_SEC     = 6.0;  // total time spent feeding

    // Park: strafe sideways. 1.0 = strafe right, -1.0 = strafe left
    static final double PARK_STRAFE_DIRECTION = 1.0;
    static final double PARK_STRAFE_SEC       = 1.0;

    // ----- Hardware -----
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    private DcMotor intake;
    private CRServo leftIntakeServo, rightIntakeServo, windmillServo;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized - waiting for start");
        telemetry.update();
        waitForStart();

        if (!opModeIsActive()) return;

        // 1) Drive to the shooting spot
        if (TO_SHOOT_SEC >= 0) {
            drive(DRIVE_POWER, 0, 0, TO_SHOOT_SEC);
        } else {
            drive(-DRIVE_POWER, 0, 0, -TO_SHOOT_SEC);
        }

        // 2) Spin up the shooter
        double readyVelocity = SHOOTER_VELOCITY * SHOOTER_READY_FRACTION;
        shooter.setVelocity(SHOOTER_VELOCITY);

        double spinStart = getRuntime();
        while (opModeIsActive()
                && shooter.getVelocity() < readyVelocity
                && getRuntime() - spinStart < SPINUP_TIMEOUT_SEC) {
            telemetry.addData("Phase", "Spinning up");
            telemetry.addData("Shooter", "%.0f / %.0f t/s", shooter.getVelocity(), SHOOTER_VELOCITY);
            telemetry.update();
            idle();
        }

        // 3) Shoot: same logic as the TeleOp, only feed while the shooter is at speed
        double shootStart = getRuntime();
        while (opModeIsActive() && getRuntime() - shootStart < SHOOT_DURATION_SEC) {
            boolean ready = shooter.getVelocity() > readyVelocity;
            windmillServo.setPower(ready ? 1 : 0);
            setIntake(ready ? 1.0 : 0);

            telemetry.addData("Phase", "Shooting");
            telemetry.addData("Shooter", "%.0f / %.0f t/s", shooter.getVelocity(), SHOOTER_VELOCITY);
            telemetry.addData("Feeding", ready);
            telemetry.update();
            idle();
        }
        windmillServo.setPower(0);
        setIntake(0);
        shooter.setVelocity(0);

        // 4) Park
        drive(0, PARK_STRAFE_DIRECTION, 0, PARK_STRAFE_SEC);

        stopDrive();
        telemetry.addData("Phase", "Done");
        telemetry.update();
    }

    private void initHardware() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");
        windmillServo    = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo  = hardwareMap.get(CRServo.class, "leftintakeservo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightintakeservo");

        // Same directions as the TeleOp
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
    }

    /*
     * Same mecanum math as the TeleOp.
     * forward: + = forward, strafe: + = right, rotate: + = clockwise (all -1..1).
     */
    private void drive(double forward, double strafe, double rotate, double seconds) {
        double lf = forward + strafe + rotate;
        double rf = forward - strafe - rotate;
        double lb = forward - strafe + rotate;
        double rb = forward + strafe - rotate;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);

        double start = getRuntime();
        while (opModeIsActive() && getRuntime() - start < seconds) {
            telemetry.addData("Phase", "Driving");
            telemetry.update();
            idle();
        }
        stopDrive();
    }

    private void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void setIntake(double power) {
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }
}