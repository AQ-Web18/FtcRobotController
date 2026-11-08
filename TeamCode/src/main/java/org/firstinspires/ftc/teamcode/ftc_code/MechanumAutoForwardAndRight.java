package org.firstinspires.ftc.teamcode.ftc_code;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Royal Robotics Guild (FTC 34324) autonomous for the goBILDA StarterBot with mecanum wheels.
 * Same drive hardware names, motor directions and mecanum math as MechanumTeleOp.
 *
 * SEQUENCE
 *   1. Drive forward 0.35 s
 *   2. Strafe right 0.35 s
 *
 * This is TIME BASED, so distances change with battery voltage.
 */
@Autonomous(name = "Mechanum Auto Move Forward And Right", group = "StarterBot")
public class MechanumAutoForwardAndRight extends LinearOpMode {

    // ----- Tunables -----
    static final double DRIVE_POWER = 0.5;
    static final double FORWARD_SEC = 0.35;
    static final double STRAFE_RIGHT_SEC = 0.5;

    // ----- Hardware -----
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized - waiting for start");
        telemetry.update();
        waitForStart();

        if (!opModeIsActive()) return;

        // 1) Forward
        drive(DRIVE_POWER, 0, 0, FORWARD_SEC);

        // 2) Strafe right (positive strafe = right)
        drive(0, DRIVE_POWER, 0, STRAFE_RIGHT_SEC);

        stopDrive();
        telemetry.addData("Phase", "Done");
        telemetry.update();
    }

    private void initHardware() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");

        // Same directions as the TeleOp
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);
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
}