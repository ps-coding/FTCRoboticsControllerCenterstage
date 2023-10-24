package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name = "AutoPark", group = "Autonomous")
public class AutoPark extends LinearOpMode {
    CSRobot robot = new CSRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized.");
        telemetry.update();
        robot.init(hardwareMap);

        waitForStart();
        robot.driveToInches(12);

//        robot.claw.setPosition(1.0);
//
//        telemetry.addData("Status", "Driving...");
//        telemetry.update();
//        robot.driveToInches(72);
//        robot.turnRight(90);
//        robot.driveToInches(24);
//        robot.turnLeft(90);
//        robot.driveToInches(10);
//
//        robot.secondaryArm.setPosition(0.8);
//
//        robot.rootArm.setPower(1 / (Math.max(1, (0.2 * Math.abs(robot.rootArm.getCurrentPosition())))));
//        try {Thread.sleep(2000);} catch (InterruptedException e) {}
//        robot.rootArm.setPower(0.0);
//
//        robot.claw.setPosition(0.0);
    }
}
