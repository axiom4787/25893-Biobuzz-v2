package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.pedropathing.ivy.commands.Commands.*;

import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.DcMotorCaching;

public class Intake extends Subsystem {
    private State state = State.IDLE;

    private final DcMotorCaching intake;

    public Intake(OpMode o) {
        super(o);

        intake = o.hardwareMap.get(DcMotorCaching.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private enum State {
        IN(1), OUT(-1), IDLE(0);

        State(int power) {
            this.power = power;
        }

        private final int power;
    }

    public Command periodicCommand() {
        return infinite(() -> {
            intake.setPower(state.power);
        });
    }

    public Command in() {
        return instant(() -> state = State.IN);
    }

    public Command out() {
        return instant(() -> state = State.OUT);
    }

    public Command idle() {
        return instant(() -> state = State.IDLE);
    }

    @Override
    public void debug() {
        o.telemetry.addData("Intake - State", state.name());
    }
}
