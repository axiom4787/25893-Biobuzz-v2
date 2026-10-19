package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad lastGamepad1 = new Gamepad();

        Robot robot = new Robot(this);

        waitForStart();

        robot.start();

        while (opModeIsActive()) {
            robot.periodic();
            robot.debug();

            // Collecting game elements from the ground: left trigger
            if (gamepad1.leftTriggerWasPressed()) {
                robot.collectGround(() -> gamepad1.leftTriggerWasReleased())
                        .schedule();
            }

            // Collecting game elements from flowers: left bumper
            if (gamepad1.leftBumperWasPressed()) {
                robot.collectFlower(() -> gamepad1.leftBumperWasPressed())
                        .schedule();
            }

            // Shoot stored game elements
            if (gamepad1.rightTriggerWasPressed()) {
                robot.shootAll().schedule();
            }

            lastGamepad1.copy(gamepad1);
        }
    }
}
