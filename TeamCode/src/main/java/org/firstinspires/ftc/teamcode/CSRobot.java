package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class CSRobot {
    private DcMotor flDrive;
    private DcMotor frDrive;
    private DcMotor blDrive;
    private DcMotor brDrive;

    public DcMotor rootArm;
    public Servo secondaryArm;
    private ElapsedTime secondaryArmDebounce = new ElapsedTime();
    private boolean secondaryArmUp = false;
    public Servo claw;
    private ElapsedTime clawDebounce = new ElapsedTime();
    private boolean clawOpen = false;

    public IMU imu;

    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    public double rootArmPower;

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");

        frDrive.setDirection(DcMotor.Direction.REVERSE);

        rootArm = hardwareMap.get(DcMotor.class, "rootArm");
        secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
        claw = hardwareMap.get(Servo.class, "claw");

        // Set up drive motors
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up root arm motor
        rootArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rootArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rootArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up servo motors
        secondaryArm.setPosition(0.0);
        claw.setPosition(1.0);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        // Core functionality
        mainDrive(gp1);

        // Plugins
        rootArmDrive(gp2);
        secondaryArmDrive(gp2);
        toggleClaw(gp2);
    }

    public void mainDrive(Gamepad gp1) {
        final double drive = (-gp1.left_stick_y);
        final double strafe = (gp1.left_stick_x);
        final double turn = (gp1.right_stick_x);

        flDrivePower = (drive + strafe + turn);
        frDrivePower = (drive - strafe - turn);
        blDrivePower = (drive - strafe + turn);
        brDrivePower = (drive + strafe - turn);

        final double SLOWDOWN = 8.0;

        flDrive.setPower(flDrivePower / SLOWDOWN);
        frDrive.setPower(frDrivePower / SLOWDOWN);
        blDrive.setPower(blDrivePower / SLOWDOWN);
        brDrive.setPower(brDrivePower / SLOWDOWN);
    }

    public void rootArmDrive(Gamepad gp2) {
        rootArmPower = gp2.left_stick_y;
        if (rootArmPower <= 0) {
            rootArm.setPower(rootArmPower / (Math.max(1, (0.2 * Math.abs(rootArm.getCurrentPosition())))));
        } else {
            rootArm.setPower(rootArmPower / 6);
        }
    }

    public void secondaryArmDrive(Gamepad gp2) {
        if (gp2.x && secondaryArmDebounce.milliseconds() > 300) {
            secondaryArmDebounce.reset();

            secondaryArmUp = !secondaryArmUp;

            if (secondaryArmUp) {
                secondaryArm.setPosition(0.8);
            } else {
                secondaryArm.setPosition(0.0);
            }
        }
    }

    public void toggleClaw(Gamepad gp2) {
        if (gp2.a && clawDebounce.milliseconds() > 300) {
            clawDebounce.reset();

            clawOpen = !clawOpen;

            if (clawOpen) {
                claw.setPosition(0.0);
            } else {
                claw.setPosition(1.0);
            }
        }
    }

    public void turnLeft(double target) {
        this.imu.resetYaw();

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;
        double lastError;

        double kp = 0.5;
        double kd = 0.1;

        final int DELAY = 50;

        while (Math.abs(error) > 1) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            lastError = error;
            error = target - currentPosition;

            double proportional = error * kp;
            double derivative = ((error - lastError) / DELAY) * kd;

            double turn = (proportional + derivative) / (180 * kp);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            flDrive.setPower(flDrivePower / 5);
            frDrive.setPower(frDrivePower / 5);
            blDrive.setPower(blDrivePower / 5);
            brDrive.setPower(brDrivePower / 5);

            try {Thread.sleep(DELAY);} catch (InterruptedException e) {}
        }
    }

    public void turnRight(double target) {
        this.imu.resetYaw();

        double currentPosition = -this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;
        double lastError;

        double kp = 0.5;
        double kd = 0.1;

        final int DELAY = 50;

        while (Math.abs(error) > 1) {
            currentPosition = -this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            lastError = error;
            error = target - currentPosition;

            double proportional = error * kp;
            double derivative = ((error - lastError) / DELAY) * kd;

            double turn = (proportional + derivative) / (180 * kp);

            flDrivePower = turn;
            frDrivePower = -turn;
            blDrivePower = turn;
            brDrivePower = -turn;

            flDrive.setPower(flDrivePower / 5);
            frDrive.setPower(frDrivePower / 5);
            blDrive.setPower(blDrivePower / 5);
            brDrive.setPower(brDrivePower / 5);

            try {Thread.sleep(DELAY);} catch (InterruptedException e) {}
        }
    }

    public void driveToInches(final double inches) {
        driveTo((int) (inches * 4.931));
    }

    private void driveTo(final int pos) {
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setTargetPos(pos);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        int direction = Math.abs(pos) / pos;
        double distance = Math.abs(pos) - Math.abs(flDrive.getCurrentPosition());
        drive(0.5 * direction * (distance / Math.abs(pos)));

        while (flDrive.isBusy() || blDrive.isBusy() || frDrive.isBusy() || brDrive.isBusy()) {
            distance = Math.abs(pos) - Math.abs(flDrive.getCurrentPosition());
            drive(0.5 * direction * (distance / Math.abs(pos)));
        }

        drive(0.0);

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDriveMode(final DcMotor.RunMode mode) {
        flDrive.setMode(mode);
        frDrive.setMode(mode);
        blDrive.setMode(mode);
        brDrive.setMode(mode);
    }

    private void setTargetPos(final int pos) {
        flDrive.setTargetPosition(pos);
        frDrive.setTargetPosition(pos);
        blDrive.setTargetPosition(pos);
        brDrive.setTargetPosition(pos);
    }

    private void drive(final double pow) {
        flDrive.setPower(pow);
        blDrive.setPower(pow);
        frDrive.setPower(pow);
        brDrive.setPower(pow);
    }
}
