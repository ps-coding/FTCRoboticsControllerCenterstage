package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CSRobot {
    private DcMotor flDrive;
    private DcMotor frDrive;
    private DcMotor blDrive;
    private DcMotor brDrive;

    private DcMotor rollMotor;

    private DcMotor rootArm;
    private Servo secondaryArm;
    private Servo claw;

    private boolean clawOpen = false;
    private ElapsedTime clawDebounce = new ElapsedTime();

    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;

    public void init(final HardwareMap hardwareMap) {
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");

        rollMotor = hardwareMap.get(DcMotor.class, "rollMotor");

        rootArm = hardwareMap.get(DcMotor.class, "rootArm");
        secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
        claw = hardwareMap.get(Servo.class, "claw");

        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        // Core functionality
        mainDrive(gp1);

        // Plugins
        rootArmDrive(gp2);
        secondaryArmDrive(gp2);
        toggleClaw(gp2);
        rollIn(gp2);
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
        final double armPower = gp2.left_stick_y;
        rootArm.setPower(armPower);
    }

    public void secondaryArmDrive(Gamepad gp2) {
        final double armPos = (gp2.right_stick_y + 1) / 2;
        // f [-1, 1]
        // + 1 = [0, 2]
        // / 2 = [0, 1]
        // -> g [0, 1]
        secondaryArm.setPosition(armPos);
    }

    public void toggleClaw(Gamepad gp2) {
        if (gp2.a && clawDebounce.milliseconds() > 300) {
            clawOpen = !clawOpen;
            clawDebounce.reset();

            if (clawOpen) {
                claw.setPosition(0.3);
            } else {
                claw.setPosition(1.0);
            }
        }
    }

    public void rollIn(Gamepad gp2) {
        if (gp2.b) {
            rollMotor.setPower(1.0);
            ElapsedTime second = new ElapsedTime();
            while (second.milliseconds() < 1000) {
                // wait
            }
            rollMotor.setPower(0.0);
        }
    }

    public void driveToInches(final double inches) {
        driveToRotations(inches * 4 * Math.PI);
    }

    public void driveToRotations(final double rotations) {
        driveTo((int) Math.floor(rotations * 288));
    }

    private void driveTo(final int pos) {
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setTargetPos(pos);
        if (pos > 0) {
            drive(0.15);
            while (flDrive.getCurrentPosition() < pos) {
                // Adjust power to each wheel to account for differences in encoder values
                double diff = ((flDrive.getCurrentPosition() - frDrive.getCurrentPosition()) * 0.0015);
                frDrive.setPower(0.15 + diff);
                diff = ((flDrive.getCurrentPosition() - blDrive.getCurrentPosition()) * 0.0015);
                blDrive.setPower(0.15 + diff);
                diff = ((flDrive.getCurrentPosition() - brDrive.getCurrentPosition()) * 0.0015);
                brDrive.setPower(0.15 + diff);
            }
        } else {
            drive(-0.15);
            while (flDrive.getCurrentPosition() > pos) {
                // Adjust power to each wheel to account for differences in encoder values
                double diff = ((flDrive.getCurrentPosition() - frDrive.getCurrentPosition()) * 0.0015);
                frDrive.setPower(-0.15 + diff);
                diff = ((flDrive.getCurrentPosition() - blDrive.getCurrentPosition()) * 0.0015);
                blDrive.setPower(-0.15 + diff);
                diff = ((flDrive.getCurrentPosition() - brDrive.getCurrentPosition()) * 0.0015);
                brDrive.setPower(-0.15 + diff);
            }
        }
        brake();
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

    private void brake() {
        drive(0.0);
    }

    private void drive(final double bothPow) {
        this.drive(bothPow, bothPow);
    }

    public void drive(final double lPow, final double rPow) {
        leftPow(lPow);
        rightPow(rPow);
    }

    private void leftPow(final double pow) {
        flDrive.setPower(pow);
        blDrive.setPower(pow);
    }

    private void rightPow(final double pow) {
        frDrive.setPower(pow);
        brDrive.setPower(pow);
    }

    public enum Wheel {
        FL,
        FR,
        BL,
        BR
    }
}
