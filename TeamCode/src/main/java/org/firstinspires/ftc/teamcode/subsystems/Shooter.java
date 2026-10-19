package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.controllers.Controller;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.DcMotorExCaching;

public class Shooter extends Subsystem {
    private final DcMotorExCaching shooter;
    private final VoltageSensor voltageSensor;

    private boolean active = true;

    private final Controller velocityController = Controller.sum(
            Controller.staticFeedforward(0.01),
            Controller.proportionalFeedforward(0.01),
            Controller.pid(0.0000, 0, 0.0)
    );

    private static final double VOLTAGE_WHEN_TUNED = 13.0;
    private static final double IDLE_POWER = 0.5;

    public Shooter(Robot r) {
        super(r);

        shooter = r.hardwareMap.get(DcMotorExCaching.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = r.hardwareMap.voltageSensor.iterator().next();
    }

    private enum State {
        DISABLED, // Power set to zero so that the shooter can't shoot
        IDLE, // Power set to 0.5 to be ready to spin up, but not using a ton of power by always being spun up
        ACTIVE; // Power sent to shooter based on distance to goal
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

    public double getTargetVelocity() {
        double distToGoal = r.drivetrain.getDistanceToGoal();
        // TODO: Find target velocity based on distance from goal
        return 0;
    }

    public double getActualVelocity() {
        return shooter.getVelocity();
    }

    public double getVelocityError() {
        return getTargetVelocity() - getActualVelocity();
    }

    @Override
    public Command periodicCommand() {
        return infinite(() -> {
            if (!active) {
                shooter.setPower(0);
            } else {
                double targetVelocity = getTargetVelocity();
                double velocityError = getVelocityError();
                double power = velocityController.calculate(targetVelocity, velocityError);

                double currentVoltage = voltageSensor.getVoltage();
                power = power * (VOLTAGE_WHEN_TUNED / currentVoltage);
                shooter.setPower(power);
            }
        });
    }
}
