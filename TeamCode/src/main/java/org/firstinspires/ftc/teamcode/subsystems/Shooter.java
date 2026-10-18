package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.controllers.Controller;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Shooter extends Subsystem {
    private final DcMotorEx shooter;
    private final VoltageSensor voltageSensor;

    private State state = State.DISABLED;
    private double distToGoal = 0.0;
    private final Controller velocityController = Controller.sum(
            Controller.staticFeedforward(0.01),
            Controller.proportionalFeedforward(0.01),
            Controller.pid(0.0000, 0, 0.0)
    );

    private static final double VOLTAGE_WHEN_TUNED = 13.0;
    private static final double IDLE_POWER = 0.5;

    public Shooter(OpMode o) {
        super(o);

        shooter = o.hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = o.hardwareMap.voltageSensor.iterator().next();
    }

    private enum State {
        DISABLED, // Power set to zero so that the shooter can't shoot
        IDLE, // Power set to 0.5 to be ready to spin up, but not using a ton of power by always being spun up
        ACTIVE; // Power sent to shooter based on distance to goal
    }

    public Command setDisabled() {
        return instant(() -> state = State.DISABLED);
    }

    public Command setIdle() {
        return instant(() -> state = State.IDLE);
    }

    public Command setActive() {
        return instant(() -> state = State.ACTIVE);
    }

    public void setDistToGoal(double distToGoal) {
        this.distToGoal = distToGoal;
    }

    @Override
    public Command periodicCommand() {
        return infinite(() -> {
            if (state == State.DISABLED) {
                shooter.setPower(0);
            } else if (state == State.IDLE) {
                shooter.setPower(IDLE_POWER);
            } else {
                // TODO: Use PID, distance to goal, and voltage compensation to set optimal power;
            }
        });
    }
}
