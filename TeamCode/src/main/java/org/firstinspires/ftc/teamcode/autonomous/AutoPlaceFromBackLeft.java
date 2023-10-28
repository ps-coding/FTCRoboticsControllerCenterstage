package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name="AutoPlaceFromBackLeft", group = "PlaceBasic")
public class AutoPlaceFromBackLeft extends LinearOpMode {
    CSRobot robot = new CSRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized.");
        telemetry.update();
        robot.init(hardwareMap);

        waitForStart();

        robot.claw.setPosition(1.0);
        robot.secondaryArm.setPosition(0.4);

        telemetry.addData("Status", "Driving...");
        telemetry.update();

        robot.tinyStrafe(1);

        robot.driveToInches(72);
        robot.tinyStrafe(6);

        robot.secondaryArm.setPosition(0.75);

        robot.rootArm.setPower(-0.3);
        try {Thread.sleep(1700);} catch (InterruptedException e) {}
        robot.rootArm.setPower(0.0);
        robot.driveToInches(8);
        try {Thread.sleep(1000);} catch (InterruptedException e) {}
        robot.claw.setPosition(0.0);
        try {Thread.sleep(2000);} catch (InterruptedException e) {}
    }
}
