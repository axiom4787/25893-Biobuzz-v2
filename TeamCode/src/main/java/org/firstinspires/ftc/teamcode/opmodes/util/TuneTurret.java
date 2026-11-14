package org.firstinspires.ftc.teamcode.opmodes.util;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor;


@Utility
public class TuneTurret extends LinearOpMode {
    private final Controller turnController =
            Controller.pid(0.8, 0, 0)
            .plus(Controller.staticFeedforward(0.05));

    private static final double CCW_LIMIT_RAD = Math.toRadians(90);
    private static final double CW_LIMIT_RAD = Math.toRadians(-130);
    private static final double MOTOR_TICKS_PER_TURRET_ROTATION;

    static {
        double TICKS_PER_MOTOR_ROTATION = 537.7; // 537.7 ticks per full drive motor rotation (312 RPM motor).
        double GEAR_RATIO = 1.0 / 3.0; // Placeholder, Input gear teeth / output gear teeth

        MOTOR_TICKS_PER_TURRET_ROTATION = TICKS_PER_MOTOR_ROTATION / GEAR_RATIO;
        // A target angle of zero should be facing straight backwards
        // Positive should be ccw,
        // Negative should be cw
    }
    private double clippedTargetAngleRadians = 0;


    CachingDcMotor turret;

    public void zero() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        turret = new CachingDcMotor(hardwareMap.get(DcMotor.class, "turret"));
        turret.setDirection(DcMotorSimple.Direction.FORWARD); // TODO: Verify direction
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zero();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                clippedTargetAngleRadians = Math.toRadians(-45);
            } else if (gamepad1.b) {
                clippedTargetAngleRadians = Math.toRadians(45);
            } else if (gamepad1.x) {
                clippedTargetAngleRadians = 0;
            }

            double power = turnController.calculate(clippedTargetAngleRadians, getErrorRadians());

            if (Math.abs(power) < 0.01) power = 0;

            turret.setPower(power);


            telemetry.addData("target", clippedTargetAngleRadians);
            telemetry.addData("actual", getActualAngleRadians());
            telemetry.addData("last power", power);
            telemetry.update();
        }
    }

    public double getActualAngleRadians() {
        double currentTicks = turret.getCurrentPosition();
        double rotations = currentTicks / MOTOR_TICKS_PER_TURRET_ROTATION;
        return rotations * Math.PI * 2;
    }

    public double getErrorRadians() {
        return clippedTargetAngleRadians - getActualAngleRadians();
    }
}
