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


public class Shooter extends Subsystem {
    private final DcMotorEx shooter;
    private final VoltageSensor voltageSensor;

    private boolean active = true;
    private double targetVelocity = 0;

    private final Controller velocityController = Controller.sum(
            Controller.staticFeedforward(0.01),
            Controller.proportionalFeedforward(0.01),
            Controller.pid(0.0000, 0, 0.0)
    );

    private static final double VOLTAGE_WHEN_TUNED = 13.0;

    public Shooter(Robot r) {
        super(r);

        shooter = r.hardwareMap.get(DcMotorEx.class, "shooter");
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
        if (!active) {
            shooter.setPower(0);
        } else {
            targetVelocity = r.sotm.getFlywheelTPS();
            double velocityError = getVelocityError();
            double power = velocityController.calculate(targetVelocity, velocityError);

            double currentVoltage = voltageSensor.getVoltage();
            power = power * (VOLTAGE_WHEN_TUNED / currentVoltage);
            shooter.setPower(power);
        }
    }

    @Override
    public void debug() {
        r.telemetry.addLine("Shooter");
        r.telemetry.addData("- Actual Velocity", "%4f", getActualVelocity());
        r.telemetry.addData("- Target Velocity", "%4f", targetVelocity);
        r.telemetry.addData("- Velocity Error", "%4f", getVelocityError());
    }
}
