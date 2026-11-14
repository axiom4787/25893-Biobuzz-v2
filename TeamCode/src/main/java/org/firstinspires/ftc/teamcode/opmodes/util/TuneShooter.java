package org.firstinspires.ftc.teamcode.opmodes.util;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.controllers.Controller;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;


@Utility
public class TuneShooter extends LinearOpMode {
    private final Controller velocityController = Controller.sum(
            Controller.staticFeedforward(0.02),
            Controller.proportionalFeedforward(0.00047),
            Controller.pid(0.005, 0, 0.0)
    );

    double targetVelocity = 0.0;
    DcMotorEx shooter;


    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter"));
        shooter.setDirection(DcMotorSimple.Direction.FORWARD); // TODO: Verify direction
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        double VOLTAGE_WHEN_TUNED = 13.6;

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                targetVelocity += 20;
            } else if (gamepad1.dpadDownWasPressed()) {
                targetVelocity -= 20;
            } else if (gamepad1.dpadLeftWasPressed()) {
                targetVelocity -= 200;
            } else if (gamepad1.dpadRightWasPressed()) {
                targetVelocity += 200;
            } else if (gamepad1.y) {
                targetVelocity = 0;
            }

            double velocityError = getVelocityError();
            double power = velocityController.calculate(targetVelocity, velocityError);

            double currentVoltage = voltageSensor.getVoltage();
            power = power * (VOLTAGE_WHEN_TUNED / currentVoltage);
            if (targetVelocity != 0) {
                shooter.setPower(power);
            } else {
                shooter.setPower(0);
            }

            telemetry.addData("target", targetVelocity);
            telemetry.addData("actual", getActualVelocity());
            telemetry.addData("last power", power);
            telemetry.update();
        }
    }

    public double getActualVelocity() {
        return shooter.getVelocity();
    }

    public double getVelocityError() {
        return targetVelocity - getActualVelocity();
    }
}
