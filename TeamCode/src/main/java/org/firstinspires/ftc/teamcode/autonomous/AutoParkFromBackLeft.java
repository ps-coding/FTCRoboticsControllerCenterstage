package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name = "AutoParkFromBackLeft", group = "Park")
public class AutoParkFromBackLeft extends LinearOpMode {
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

        robot.driveToInches(94);
    }
}
