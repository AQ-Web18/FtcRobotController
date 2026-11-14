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
 * Same hardware names, motor directions and shooter PIDF as the TeleOp.
 *
 * SEQUENCE
 *   0. Wait 10 s
 *   1. Spin the shooter to SHOOTER_VELOCITY; once it gets there the windmill (and intake) start and shoot
 *   2. Backward 0.50 s
 *   3. Turn left (counterclockwise) 90 degrees
 *   4. Forward 1.50 s
 *
 * This is TIME BASED, so distances change with battery voltage. Tune the constants below
 * on a fully charged battery.
 */
@Autonomous(name = "Mechanum Auto Shooting 2", group = "StarterBot")

public class MechanumAuto2 extends LinearOpMode {

    // ----- Tunables -----
    static final double START_DELAY_SEC = 10.0;  // wait before doing anything

    static final double DRIVE_POWER = 0.5;
    static final double TURN_POWER  = 0.5;

    // Moves after shooting
    static final double BACKWARD_SEC  = 0.50;
    static final double TURN_LEFT_SEC = 0.75;  // TUNE THIS: time for a 90 degree left (CCW) turn
    static final double FORWARD_SEC   = 1.25;

    // Shooter (ticks per second, same units as the TeleOp)
    static final double SHOOTER_VELOCITY   = 1230;
    static final double SPINUP_TIMEOUT_SEC = 3.0;  // feed anyway if it never reaches SHOOTER_VELOCITY
    static final double SHOOT_DURATION_SEC = 8.0;  // total time spent feeding
    static final double INTAKE_POWER       = 1.0;  // intake runs along with the windmill

    // ----- Hardware -----
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooter;
    private CRServo windmillServo;
    private DcMotor intake;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized - waiting for start");
        telemetry.update();
        waitForStart();

        if (!opModeIsActive()) return;

        // 0) Initial delay before doing anything
        double delayStart = getRuntime();
        while (opModeIsActive() && getRuntime() - delayStart < START_DELAY_SEC) {
            telemetry.addData("Phase", "Waiting to start");
            telemetry.addData("Time left", "%.1f s", START_DELAY_SEC - (getRuntime() - delayStart));
            telemetry.update();
            idle();
        }

        // 1) Spin the shooter up to the target speed
        shooter.setVelocity(SHOOTER_VELOCITY);

        double spinStart = getRuntime();
        while (opModeIsActive()
                && shooter.getVelocity() < SHOOTER_VELOCITY
                && getRuntime() - spinStart < SPINUP_TIMEOUT_SEC) {
            telemetry.addData("Phase", "Spinning up");
            telemetry.addData("Shooter", "%.0f / %.0f t/s", shooter.getVelocity(), SHOOTER_VELOCITY);
            telemetry.update();
            idle();
        }

        // Windmill starts once the shooter is up to speed and stays on for the shoot time
        windmillServo.setPower(1);
        intake.setPower(INTAKE_POWER);
        double shootStart = getRuntime();
        while (opModeIsActive() && getRuntime() - shootStart < SHOOT_DURATION_SEC) {
            telemetry.addData("Phase", "Shooting");
            telemetry.addData("Shooter", "%.0f / %.0f t/s", shooter.getVelocity(), SHOOTER_VELOCITY);
            telemetry.update();
            idle();
        }
        windmillServo.setPower(0);
        intake.setPower(0);
        shooter.setVelocity(0);

        // 2) Backward
        drive("Backward", -DRIVE_POWER, 0, 0, BACKWARD_SEC);

        // 3) Turn left 90 degrees (negative rotate = counterclockwise)
        drive("Turn left 90", 0, 0, -TURN_POWER, TURN_LEFT_SEC);

        // 4) Forward
        drive("Forward", DRIVE_POWER, 0, 0, FORWARD_SEC);

        stopDrive();
        telemetry.addData("Phase", "Done");
        telemetry.update();
    }

    private void initHardware() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        intake     = hardwareMap.get(DcMotor.class, "intake");  // CHECK: must match your config name

        // Same directions as the TeleOp
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setPower(0);

        intake.setDirection(DcMotor.Direction.FORWARD);  // flip to REVERSE if it spins the wrong way
        intake.setPower(0);
    }

    /*
     * Same mecanum math as the TeleOp.
     * forward: + = forward, strafe: + = right, rotate: + = clockwise (all -1..1).
     */
    private void drive(String label, double forward, double strafe, double rotate, double seconds) {
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
            telemetry.addData("Phase", label);
            telemetry.addData("Time left", "%.2f s", seconds - (getRuntime() - start));
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
}