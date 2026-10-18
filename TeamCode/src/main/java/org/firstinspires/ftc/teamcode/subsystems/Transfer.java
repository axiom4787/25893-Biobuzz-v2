package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Transfer extends Subsystem {
    private final DcMotor transfer;

    private State state = State.IDLE;
    private double lastPower = state.power;

    public Transfer(OpMode o) {
        super(o);

        transfer = o.hardwareMap.get(DcMotor.class, "transfer");
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
            if (lastPower != state.power) {
                transfer.setPower(state.power);
                lastPower = state.power;
            }
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
        o.telemetry.addData("Transfer - State", state.name());
    }
}
