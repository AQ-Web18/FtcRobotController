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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
/*
 * ROYAL ROBOTICS GUILD - FTC 34324
 *
 * One OpMode that contains BOTH the StarterBot teleop and the StarterBot autonomous routine.
 *
 *   D-PAD UP (gamepad 1) toggles between the two modes. Our controller's M1 back paddle is
 *   programmed (on the controller itself) to copy D-pad up, so pressing M1 does the same thing.
 *   The SDK can't see M1 directly; it only sees the D-pad up press that M1 sends.
 *
 *     DRIVER mode -> normal teleop driving (arcade drive, triggers = intake, R1 = launch,
 *                    L1 = turn the windmill by hand to load/index elements)
 *     AUTO mode   -> runs the autonomous routine from the start (launch for 10 s, then back up 120 mm)
 *
 *   Pressing M1 / D-PAD UP again at any time during AUTO cancels it and gives control back to the
 *   driver, so it doubles as an "abort" button.
 *
 * D-pad up was chosen because the original teleop does not use it.
 *
 * NOTE: During a real match, the Driver Station ignores gamepads in the autonomous period,
 * so this file is for practice and demos. Keep the separate @Autonomous OpMode for competition.
 */

@TeleOp(name = "RRG StarterBot (Driver + Auto)")
//@Disabled
public class StarterBot1 extends OpMode {

    // ---------------------------------------------------------------- Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // ---------------------------------------------------------------- Launcher settings
    /*
     * Launcher velocities in encoder ticks per second (28 ticks per revolution on our motors).
     * The windmill only feeds once the launcher is above LAUNCHER_MIN_VELOCITY.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    // ---------------------------------------------------------------- Auto drive settings
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 402;
    final double AUTO_LAUNCH_SECONDS = 10;

    /*
     * Windmill power used when the driver holds L1 to load elements by hand (0-1).
     * Lower = slower and more controlled. Raise it if loading feels too slow.
     */
    final double WINDMILL_LOAD_POWER = 0.5;

    // ---------------------------------------------------------------- Shared variables
    double leftPower;
    double rightPower;
    double intakePower = 0;

    /*
     * Like intakePower, we figure out what the windmill should do first, then set it ONCE at the
     * end of each loop. Setting the same servo twice in one loop can make it jitter.
     */
    double windmillPower = 0;

    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();

    // ---------------------------------------------------------------- State machines
    /*
     * RobotMode is the "big" state machine: is the driver in control, or is the auto routine?
     * AutonomousState is the original auto's state machine, used only while in AUTO mode.
     */
    private enum RobotMode {
        DRIVER,
        AUTO
    }

    private enum AutonomousState {
        LAUNCH,
        DRIVE,
        COMPLETE
    }

    private RobotMode robotMode = RobotMode.DRIVER;
    private AutonomousState autonomousState = AutonomousState.LAUNCH;

    /*
     * Remembers whether the mode button was held during the previous loop. We only toggle when
     * the button goes from "not pressed" to "pressed" (a rising edge). Without this, holding the
     * button for even a fraction of a second would flip modes dozens of times.
     */
    private boolean lastModeButton = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftdrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightdrive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftintakeservo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightintakeservo");

        // Pushing the left stick forward MUST make the robot go forward. Flip these if it doesn't.
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Start in driver mode: drive motors take raw power from the joysticks.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Closed-loop velocity control for the launcher, with PIDF tuned for a flywheel.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        robotMode = RobotMode.DRIVER;

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("M1 (D-PAD UP) = switch between DRIVER and AUTO");
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
        // ---- Check the mode-switch button: M1 paddle / D-pad up (rising edge only)
        boolean modeButtonNow = gamepad1.dpad_up;
        if (modeButtonNow && !lastModeButton) {
            if (robotMode == RobotMode.DRIVER) {
                enterAutoMode();
            } else {
                enterDriverMode();
            }
        }
        lastModeButton = modeButtonNow;

        // ---- Run whichever mode we're in
        switch (robotMode) {
            case DRIVER:
                driverLoop();
                break;
            case AUTO:
                autoLoop();
                break;
        }

        telemetry.addData("MODE", robotMode + "   (M1 / D-PAD UP to switch)");
    }

    @Override
    public void stop() {
    }

    // =============================================================== Mode switching

    /*
     * Gets everything ready to run the auto routine from the very beginning.
     * The drive encoders are reset so the auto's distances are measured from where the
     * robot is sitting right now.
     */
    void enterAutoMode() {
        stopAllMechanisms();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        autonomousState = AutonomousState.LAUNCH;
        intakePower = 0;
        autoTimer.reset();
        driveTimer.reset();

        robotMode = RobotMode.AUTO;
    }

    /*
     * Hands control back to the driver. Drive motors go back to taking raw power,
     * and everything is stopped so nothing keeps running from the auto routine.
     */
    void enterDriverMode() {
        stopAllMechanisms();

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakePower = 0;
        robotMode = RobotMode.DRIVER;
    }

    void stopAllMechanisms() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        launcher.setVelocity(0);
        windmillServo.setPower(0);
        windmillPower = 0;
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    // =============================================================== DRIVER mode (teleop)

    void driverLoop() {
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        // R2 = intake in, L2 = intake out.
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        // L1 (left bumper) = turn the windmill by hand to load elements.
        windmillPower = 0;
        if (gamepad1.left_bumper) {
            windmillPower = WINDMILL_LOAD_POWER;
        }

        // R1 (right bumper) spins up the launcher and feeds once it's at speed.
        // launch() may change intakePower and windmillPower, so it must run BEFORE we set them.
        // If R1 and L1 are both held, launching wins and the windmill runs at full feed speed.
        launch(gamepad1.right_bumper);

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
        windmillServo.setPower(windmillPower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Launcher velocity", "%.0f", launcher.getVelocity());
        telemetry.addData("Windmill", "%.2f", windmillPower);
    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    // =============================================================== AUTO mode

    void autoLoop() {
        windmillPower = 0;

        switch (autonomousState) {
            case LAUNCH:
                launch(true);
                if (autoTimer.seconds() > AUTO_LAUNCH_SECONDS) {
                    launch(false);
                    intakePower = 0;
                    autonomousState = AutonomousState.DRIVE;
                }
                break;
            case DRIVE:
                if (drive(DRIVE_SPEED, -120, DistanceUnit.MM, 2)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
            case COMPLETE:
                telemetry.addLine("Auto Complete! Press M1 (D-PAD UP) to drive.");
                break;
        }

        intake.setPower(intakePower);
        windmillServo.setPower(windmillPower);

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Auto time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Motor Current Positions", "left (%d), right (%d)",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Motor Target Positions", "left (%d), right (%d)",
                leftDrive.getTargetPosition(), rightDrive.getTargetPosition());
    }

    // =============================================================== Shared launcher

    /*
     * Used by BOTH modes. In DRIVER mode we pass in the right bumper (R1); in AUTO we pass in
     * true/false. Spins the launcher to target velocity, and only feeds with the windmill once
     * it's fast enough. This sets windmillPower; the actual servo is set at the end of the loop.
     */
    void launch(boolean shouldLaunch) {
        if (shouldLaunch) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (shouldLaunch && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillPower = 1;
            intakePower += 0.5;
        }
    }

    // =============================================================== Auto drive helpers

    /**
     * @param speed From 0-1
     * @param distance In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds seconds to wait at position before returning true.
     * @return true once the motors have been within tolerance of the target for holdSeconds.
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        leftDrive.setTargetPosition((int) targetPosition);
        rightDrive.setTargetPosition((int) targetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        if (Math.abs(targetPosition - leftDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * @param speed From 0-1
     * @param angle the amount that the robot should rotate
     * @param angleUnit the unit that angle is in
     * @param holdSeconds seconds to wait at position before returning true.
     * @return true once the motors have been within tolerance of the target for holdSeconds.
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);

        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        leftDrive.setTargetPosition((int) leftTargetPosition);
        rightDrive.setTargetPosition((int) rightTargetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        if ((Math.abs(leftTargetPosition - leftDrive.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}