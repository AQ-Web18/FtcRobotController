package org.firstinspires.ftc.teamcode.ftc_code;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Royal Robotics Guild (FTC 34324) teleop for the goBILDA StarterBot with mecanum wheels.
 *
 * CONTROLS (gamepad1)
 *   Right stick up/down ..... drive forward / backward
 *   Right stick left/right .. strafe left / right
 *   Left stick left/right ... rotate left / right
 *   Right trigger ........... intake in (proportional)
 *   Left trigger ............ intake out / reverse (proportional)
 *   Right bumper (hold) ..... spin up shooter; feeds automatically once at speed
 *   D-pad right ............. shooter speed UP   (one step per press)
 *   D-pad left .............. shooter speed DOWN (one step per press)
 *
 * The shooter uses closed-loop velocity control (RUN_USING_ENCODER). Velocities are in
 * encoder ticks per second. These motors report 28 ticks per revolution, so
 * RPM = ticks/sec / 28 * 60.
 */

@TeleOp(name = "Mechanum Teleop", group = "StarterBot")
//@Disabled
public class MechanumTeleOp extends OpMode {

    // ----- Hardware -----
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotorEx shooter = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // ----- Shooter speed settings (ticks per second) -----
    public static final double SHOOTER_START_VELOCITY = 1250; // ~2678 RPM
    public static final double SHOOTER_STEP           = 50;   // change per D-pad press (~107 RPM)
    public static final double SHOOTER_MIN_VELOCITY   = 500;  // lowest allowed setting (~1071 RPM)
    public static final double SHOOTER_MAX_VELOCITY   = 2600; // near the 6000 RPM motor's top speed (~5571 RPM)

    /*
     * The windmill only feeds once the shooter reaches this fraction of the target speed.
     * The original code used 1200 out of 1250, which is 96%. Using a percentage means the
     * "ready to fire" check still works after you change the speed with the D-pad.
     */
    public static final double SHOOTER_READY_FRACTION = 0.96;

    // Current target speed, adjustable during the match with the D-pad
    private double shooterTargetVelocity = SHOOTER_START_VELOCITY;

    // Previous D-pad states, used so one press = one step (not one step every loop)
    private boolean prevDpadRight = false;
    private boolean prevDpadLeft = false;

    // Drive powers stored as fields so telemetry can show them
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    double intakePower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * These names MUST match the names in the robot configuration on the Driver Station.
         */
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        shooter    = hardwareMap.get(DcMotorEx.class, "shooter");
        windmillServo    = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo  = hardwareMap.get(CRServo.class, "leftintakeservo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightintakeservo");

        /*
         * Each line makes positive power spin that wheel so the robot moves FORWARD.
         * These were set from testing on our robot: with the stock settings, left_front,
         * right_front and right_back spun backward, so those three were flipped.
         * If a wheel spins the wrong way, flip ONLY that wheel's line.
         * Use the "Drive Motor Test" OpMode to check one wheel at a time.
         */
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Brake mode makes the robot stop quickly when the sticks are released
        leftFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Shooter uses closed-loop velocity control. If the speed jumps far above the target,
         * check that the encoder cable is in the port next to the motor's power port.
         */
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        // Servo directions so both intake servos pull game elements in
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Start all servos stopped
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Code version", "v3 - drive directions fixed");
        telemetry.addData("Shooter start speed", "%.0f t/s (%.0f RPM)",
                shooterTargetVelocity, ticksToRpm(shooterTargetVelocity));
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Right stick: forward/back and strafe. Left stick X: rotate.
         * Pushing a stick forward gives a NEGATIVE y value, so we flip the sign.
         */
        double forward = -gamepad1.right_stick_y;
        double strafe  =  gamepad1.right_stick_x;
        double rotate  =  gamepad1.left_stick_x;
        mecanumDrive(forward, strafe, rotate);

        // Intake: right trigger pulls in, left trigger pushes out
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        adjustShooterSpeed();
        launch();

        /*
         * Set intake power AFTER launch(), because launch() may add extra intake power
         * to help push elements into the shooter.
         */
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        // ----- Telemetry -----
        telemetry.addData("Shooter target", "%.0f t/s (%.0f RPM)",
                shooterTargetVelocity, ticksToRpm(shooterTargetVelocity));
        telemetry.addData("Shooter actual", "%.0f t/s (%.0f RPM)",
                shooter.getVelocity(), ticksToRpm(shooter.getVelocity()));
        telemetry.addData("Front", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Back", "left (%.2f), right (%.2f)", leftBackPower, rightBackPower);
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)",
                gamepad1.left_trigger, gamepad1.right_trigger);
    }

    @Override
    public void stop() {
    }

    /*
     * Standard mecanum math. If any wheel power goes above 1.0, all four are scaled down
     * together so the robot keeps moving in the direction the driver asked for.
     */
    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower  = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower   = forward - strafe + rotate;
        rightBackPower  = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    /*
     * D-pad right = faster, D-pad left = slower.
     * We compare to last loop's button state so the speed changes only once per press.
     * Without this, holding the button for half a second would change the speed ~25 times.
     */
    void adjustShooterSpeed() {
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadLeft  = gamepad1.dpad_left;

        if (dpadRight && !prevDpadRight) {
            shooterTargetVelocity += SHOOTER_STEP;
        }
        if (dpadLeft && !prevDpadLeft) {
            shooterTargetVelocity -= SHOOTER_STEP;
        }

        // Keep the target inside safe limits
        shooterTargetVelocity = Math.max(SHOOTER_MIN_VELOCITY,
                Math.min(SHOOTER_MAX_VELOCITY, shooterTargetVelocity));

        prevDpadRight = dpadRight;
        prevDpadLeft  = dpadLeft;
    }

    /*
     * Hold right bumper to spin up the shooter. Once it reaches 96% of the target speed,
     * the windmill feeds elements in and the intake gets a little boost to clear jams.
     */
    void launch() {
        if (gamepad1.right_bumper) {
            shooter.setVelocity(shooterTargetVelocity);
        } else {
            shooter.setVelocity(0);
        }

        double readyVelocity = shooterTargetVelocity * SHOOTER_READY_FRACTION;

        if (gamepad1.right_bumper && shooter.getVelocity() > readyVelocity) {
            windmillServo.setPower(1);
            intakePower += 1;
        } else {
            windmillServo.setPower(0);
        }
    }

    // Converts encoder ticks per second to RPM (28 ticks per revolution)
    private double ticksToRpm(double ticksPerSecond) {
        return ticksPerSecond / 28.0 * 60.0;
    }
}
