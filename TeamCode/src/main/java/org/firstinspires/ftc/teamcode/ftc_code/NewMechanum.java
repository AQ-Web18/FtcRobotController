package org.firstinspires.ftc.teamcode.ftc_code;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

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

/*
 * ROYAL ROBOTICS GUILD - FTC 34324
 *
 * StarterBot teleop + autonomous in ONE OpMode, now using a MECANUM (strafer) drivetrain.
 *
 *   D-PAD UP (gamepad 1) toggles between the two modes (M1 paddle is mapped to D-pad up).
 *
 *     DRIVER mode -> mecanum driving:
 *                      left stick  = forward/back + strafe left/right
 *                      right stick = turn
 *                    triggers = intake, R1 = launch, L1 = turn the windmill by hand
 *     AUTO mode   -> launch for 10 s, then back up 120 mm
 *
 *   Pressing D-PAD UP again during AUTO cancels it and returns control to the driver.
 *
 * NOTE: During a real match the Driver Station ignores gamepads in the autonomous period,
 * so this file is for practice and demos. Keep a separate @Autonomous OpMode for competition.
 */

@TeleOp(name = "RRG StarterBot Mecanum (Driver + Auto)")
//@Disabled
public class NewMechanum extends OpMode {

    // ---------------------------------------------------------------- Hardware
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // ---------------------------------------------------------------- Launcher settings
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

    // Mecanum wheels slip a little when strafing; this multiplier compensates.
    final double STRAFE_MULTIPLIER = 1.1;

    final double WINDMILL_LOAD_POWER = 0.5;

    // ---------------------------------------------------------------- Shared variables
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    double intakePower = 0;
    double windmillPower = 0;

    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();

    // ---------------------------------------------------------------- State machines
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

    private boolean lastModeButton = false;

    @Override
    public void init() {
        // Names must match the REV Control/Expansion Hub configuration.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftintakeservo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightintakeservo");

        // Directions copied from the original Mecanum code. Positive power on ALL four motors
        // should drive the robot forward. If it doesn't, flip the wrong ones.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    @Override
    public void loop() {
        boolean modeButtonNow = gamepad1.dpad_up;
        if (modeButtonNow && !lastModeButton) {
            if (robotMode == RobotMode.DRIVER) {
                enterAutoMode();
            } else {
                enterDriverMode();
            }
        }
        lastModeButton = modeButtonNow;

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

    // =============================================================== Drive helpers

    void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    void setDrivePower(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    void setDriveTargets(int lf, int lb, int rf, int rb) {
        leftFrontDrive.setTargetPosition(lf);
        leftBackDrive.setTargetPosition(lb);
        rightFrontDrive.setTargetPosition(rf);
        rightBackDrive.setTargetPosition(rb);
    }

    // =============================================================== Mode switching

    void enterAutoMode() {
        stopAllMechanisms();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        autonomousState = AutonomousState.LAUNCH;
        intakePower = 0;
        autoTimer.reset();
        driveTimer.reset();

        robotMode = RobotMode.AUTO;
    }

    void enterDriverMode() {
        stopAllMechanisms();

        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakePower = 0;
        robotMode = RobotMode.DRIVER;
    }

    void stopAllMechanisms() {
        setDrivePower(0);
        launcher.setVelocity(0);
        windmillServo.setPower(0);
        windmillPower = 0;
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    // =============================================================== DRIVER mode (teleop)

    void driverLoop() {
        // Left stick = move (forward/back + strafe), right stick X = turn.
        mecanumDrive(-gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);

        // R2 = intake in, L2 = intake out.
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        // L1 = turn the windmill by hand to load elements.
        windmillPower = 0;
        if (gamepad1.left_bumper) {
            windmillPower = WINDMILL_LOAD_POWER;
        }

        // R1 spins up the launcher and feeds once at speed. Must run BEFORE we set the powers.
        launch(gamepad1.right_bumper);

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
        windmillServo.setPower(windmillPower);

        telemetry.addData("Front", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);
        telemetry.addData("Back", "left (%.2f), right (%.2f)", backLeftPower, backRightPower);
        telemetry.addData("Launcher velocity", "%.0f", launcher.getVelocity());
        telemetry.addData("Windmill", "%.2f", windmillPower);
    }

    /**
     * @param forward  +1 = forward
     * @param strafe   +1 = right
     * @param rotate   +1 = turn right
     */
    void mecanumDrive(double forward, double strafe, double rotate) {
        double y = forward;
        double x = strafe * STRAFE_MULTIPLIER;
        double rx = rotate;

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
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
        windmillServo.setPower(windmillPower);

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Auto time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Current Pos (front)", "left (%d), right (%d)",
                leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
        telemetry.addData("Current Pos (back)", "left (%d), right (%d)",
                leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        telemetry.addData("Target Pos (front)", "left (%d), right (%d)",
                leftFrontDrive.getTargetPosition(), rightFrontDrive.getTargetPosition());
    }

    // =============================================================== Shared launcher

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
     * Drive straight forward/backward using all four wheels.
     * @return true once the robot has been within tolerance of the target for holdSeconds.
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);
        int target = (int) targetPosition;

        setDriveTargets(target, target, target, target);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(speed);

        if (Math.abs(targetPosition - leftFrontDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Rotate in place using all four wheels.
     * @return true once the robot has been within tolerance of the target for holdSeconds.
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);

        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        setDriveTargets((int) leftTargetPosition, (int) leftTargetPosition,
                (int) rightTargetPosition, (int) rightTargetPosition);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(speed);

        if (Math.abs(leftTargetPosition - leftFrontDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}