package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CSRobot {
    private DcMotor flDrive;
    private DcMotor frDrive;
    private DcMotor blDrive;
    private DcMotor brDrive;

    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;

    public void init(final HardwareMap hardwareMap) {
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        double drive = (-gp1.left_stick_y);
        double strafe = (gp1.left_stick_x);
        double turn = (gp1.right_stick_x);

        flDrivePower = (drive + strafe + turn);
        frDrivePower = (drive - strafe - turn);
        blDrivePower = (drive - strafe + turn);
        brDrivePower = (drive + strafe - turn);

        final double SLOWDOWN = 8.0;
        blDrive.setPower(blDrivePower / SLOWDOWN);
        brDrive.setPower(brDrivePower / SLOWDOWN);
        flDrive.setPower(flDrivePower / SLOWDOWN);
        frDrive.setPower(frDrivePower / SLOWDOWN);
    }

    public void driveToInches(final double inches) {
        driveToRotations(inches * 4 * Math.PI);
    }

    public void driveWheel(Wheel wheel) {
        switch (wheel) {
            case FL:
                flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    private void driveToRotations(final double rotations) {
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

    private void drive(final double bothPow) { // override of drive(double, double)
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
