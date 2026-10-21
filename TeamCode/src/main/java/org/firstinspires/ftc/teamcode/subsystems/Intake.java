package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.DcMotorCaching;


public class Intake extends Subsystem {
    private State state = State.IDLE;

    private final DcMotorCaching intake;

    public Intake(Robot r) {
        super(r);

        intake = r.hardwareMap.get(DcMotorCaching.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private enum State {
        IN(1), OUT(-1), IDLE(0);

        State(int power) {
            this.power = power;
        }

        private final int power;
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
    public void periodic() {
        intake.setPower(state.power);
    }

    @Override
    public void debug() {
        r.telemetry.addData("Intake - State", state.name());
    }
}
