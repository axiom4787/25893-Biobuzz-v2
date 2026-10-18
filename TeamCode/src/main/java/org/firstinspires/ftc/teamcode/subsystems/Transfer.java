package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.DcMotorCaching;

public class Transfer extends Subsystem {
    private final DcMotorCaching transfer;

    private State state = State.IDLE;

    public Transfer(Robot r) {
        super(r);

        transfer = r.hardwareMap.get(DcMotorCaching.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
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
            transfer.setPower(state.power);
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
        r.telemetry.addData("Transfer - State", state.name());
    }
}
