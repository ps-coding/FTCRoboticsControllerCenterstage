package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name="AutoPlaceFromFrontRight", group = "PlaceBasic")
public class AutoPlaceFromFrontRight extends LinearOpMode {
    CSRobot robot = new CSRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized.");
        telemetry.update();
        robot.autoInit(hardwareMap);

        waitForStart();

        robot.claw.setPosition(1.0);
        robot.secondaryArm.setPosition(0.4);

        Thread.sleep(2000);

        telemetry.addData("Status", "Driving...");
        telemetry.update();

        robot.tinyStrafe(-1);

        robot.driveToInches(14);
        robot.tinyStrafe(-6);

        robot.secondaryArm.setPosition(0.8);

        ElapsedTime wait = new ElapsedTime();
        while (wait.milliseconds() < 1550) {
            robot.rootArm.setPower(-0.7 / (Math.max(1, (0.4 * Math.abs(robot.rootArm.getCurrentPosition())))));
        }
        robot.rootArm.setPower(0.0);

        robot.driveToInches(14);
        try {Thread.sleep(1000);} catch (InterruptedException e) {}
        robot.claw.setPosition(0.0);
        try {Thread.sleep(2000);} catch (InterruptedException e) {}
    }
}
