package org.firstinspires.ftc.teamcode.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.util.hardware.DcMotorCaching;

@Utility(name = "Test: Caching hardware")
public class TestCachingHardware extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorCaching motor = hardwareMap.get(DcMotorCaching.class, "motor");
        motor.setPower(0);
        motor.setCachingThreshold(1000);

        waitForStart();

        while (opModeIsActive()) {
            // Motor should NOT start moving, as 0.5 is not within
            // the caching threshold of the last write (0)
            if (gamepad1.right_trigger_pressed) {
                motor.setPower(0.5);
            } else {
                motor.setPower(0);
            }
        }
    }
}
