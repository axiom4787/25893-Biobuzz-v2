package org.firstinspires.ftc.teamcode.opmodes.util;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.FlowerDescorer;


@Utility
public class TestFlowerDescorer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo left = hardwareMap.get(Servo.class, "leftFlower");
        Servo right = hardwareMap.get(Servo.class, "rightFlower");
        right.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down);
        }
    }
}
