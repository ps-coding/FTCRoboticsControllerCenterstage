package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CSRobot {
    private DcMotor frDrive;
    private DcMotor flDrive;
    private DcMotor brDrive;
    private DcMotor blDrive;

    private double drive;
    private double strafe;
    private double turn;

    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();

    public double PIDControl() {
        return 1;
    }
}
