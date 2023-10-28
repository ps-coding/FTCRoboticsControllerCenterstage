package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name="AutoPlaceFromBackRight", group = "PlaceBasic")
public class AutoPlaceFromBackRight extends LinearOpMode {
    CSRobot robot = new CSRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized.");
        telemetry.update();
        robot.init(hardwareMap);

        waitForStart();

        robot.claw.setPosition(1.0);

        telemetry.addData("Status", "Driving...");
        telemetry.update();

        robot.tinyStrafe(-1);

        robot.driveToInches(72);
        robot.tinyStrafe(-6);

        robot.secondaryArm.setPosition(0.8);

        robot.rootArm.setPower(-0.3);
        try {Thread.sleep(1700);} catch (InterruptedException e) {}
        robot.rootArm.setPower(0.0);
        robot.driveToInches(5);
        try {Thread.sleep(1000);} catch (InterruptedException e) {}
        robot.claw.setPosition(0.0);
        try {Thread.sleep(2000);} catch (InterruptedException e) {}
    }
}
