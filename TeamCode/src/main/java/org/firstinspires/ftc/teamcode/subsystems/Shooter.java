package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.controllers.Controller;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;


public class Shooter extends Subsystem {
    private final CachingDcMotorEx shooter;
    private final VoltageSensor voltageSensor;

    private boolean active = true;
    private double targetVelocity = 0;

    private final Controller velocityController = Controller.sum(
            Controller.staticFeedforward(0.02),
            Controller.proportionalFeedforward(0.00047),
            Controller.pid(0.005, 0, 0.0)
    );

    private static final double VOLTAGE_WHEN_TUNED = 13.6;

    public Shooter(Robot r) {
        super(r);

        shooter = new CachingDcMotorEx(r.hardwareMap.get(DcMotorEx.class, "shooter"));
        shooter.setDirection(DcMotorSimple.Direction.FORWARD); // TODO: Verify direction
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = r.hardwareMap.voltageSensor.iterator().next();
    }

    public Command disable() {
        return instant(() -> active = false);
    }

    public Command enable() {
        return instant(() -> active = true);
    }

    public boolean isReady() {
        return getVelocityError() < 150;
    }

    public double getActualVelocity() {
        return shooter.getVelocity();
    }

    public double getVelocityError() {
        return targetVelocity - getActualVelocity();
    }

    @Override
    public void periodic() {
        double slope = (1160 - 1040.0) / (60 - 35);
        double t = 0;
        double dist = r.drivetrain.getDistanceToGoal();
        if (dist <= 35) {
            t = 1040;
        } else if (dist >= 60) {
            t = 1160;
        } else {
            t = slope * (dist - 35) + 1040;
        }
        targetVelocity = t;

        if (!active) {
            shooter.setPower(0);
        } else {
//            if (r.gamepad.dpadUpWasPressed()) {
//                targetVelocity += 20;
//            } else if (r.gamepad.dpadDownWasPressed()) {
//                targetVelocity -= 20;
//            } else if (r.gamepad.dpadLeftWasPressed()) {
//                targetVelocity -= 200;
//            } else if (r.gamepad.dpadRightWasPressed()) {
//                targetVelocity += 200;
//            } else if (r.gamepad.y) {
//                targetVelocity = 0;
//            }

//            targetVelocity = r.sotm.getFlywheelTPS();
//            targetVelocity = 1120;

            double velocityError = getVelocityError();
            double power = velocityController.calculate(targetVelocity, velocityError);

            double currentVoltage = voltageSensor.getVoltage();
            power = power * (VOLTAGE_WHEN_TUNED / currentVoltage);
            if (targetVelocity == 0) {
                shooter.setPower(0);
            } else {
                shooter.setPower(power);
            }
        }

        r.telemetry.addData("dist to goal", r.drivetrain.getDistanceToGoal());
    }

    @Override
    public void debug() {
        r.telemetry.addLine("Shooter");
        r.telemetry.addData("- Actual Velocity", "%4f", getActualVelocity());
        r.telemetry.addData("- Target Velocity", "%4f", targetVelocity);
        r.telemetry.addData("- Velocity Error", "%4f", getVelocityError());
    }
}
