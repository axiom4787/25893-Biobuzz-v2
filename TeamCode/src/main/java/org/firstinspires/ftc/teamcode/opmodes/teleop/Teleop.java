package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        waitForStart();

        robot.start();

        while (opModeIsActive()) {
            // Update subsystems and log to driver station
            robot.periodic();
            robot.debug();

            // Field centric driving
            robot.drivetrain.drive(ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    robot.drivetrain.getPose().heading()
            ), false);


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
                if (robot.drivetrain.isInScoringZone()
                        && Math.abs(robot.turret.getErrorDegrees()) < 12
                        && robot.shooter.isReady()
                ) {
                    robot.shootAll().schedule();
                }
            }
        }
    }
}
