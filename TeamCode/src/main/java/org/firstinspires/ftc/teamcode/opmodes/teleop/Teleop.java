package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        waitForStart();

        robot.start();
        robot.usePreviousOpModePose();
        robot.turret.enable().schedule();
//        robot.drivetrain.setPose();

        while (opModeIsActive()) {
            // Update subsystems and log to driver station
            robot.periodic();
            robot.debug();

            // Field centric driving
            robot.drivetrain.drive(ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    robot.drivetrain.getPose().heading()
            ), false);


            // Collecting game elements from the ground: left trigger
            if (gamepad1.left_trigger >= 0.2) {
                robot.collectGround()
                        .until(() -> gamepad1.left_trigger < 0.2)
                        .schedule();
            }

            // Collecting game elements from flowers: left bumper
            if (gamepad1.left_bumper) {
                robot.collectFlower()
                        .until(() -> !gamepad1.left_bumper)
                        .schedule();
            }

            // Reverse the intake in case of nectar
            if (gamepad1.right_bumper) {
                robot.uncollect()
                        .until(() -> !gamepad1.right_bumper)
                        .schedule();
            }

            // Shoot stored game elements: right trigger
            if (gamepad1.right_trigger >= 0.2) { //  && Math.abs(robot.turret.getErrorDegrees()) < 10
                robot.shoot()
                        .until(() -> gamepad1.right_trigger < 0.2)
                        .schedule();
            }

            if (gamepad1.a) {
                robot.drivetrain.setPose(Robot.RELOCALIZATION_PARK_CORNER);
            }

            if (gamepad1.y) {
                robot.turret.zero_().schedule();
            }
        }
    }
}
