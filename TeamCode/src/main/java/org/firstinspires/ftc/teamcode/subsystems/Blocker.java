package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Blocker extends Subsystem {
    private final Servo blocker;

    private static final double BLOCKING_POSITION = 0.2;
    private static final double UNBLOCKING_POSITION = 0.8;

    public Blocker(OpMode o) {
        super(o);

        blocker = o.hardwareMap.get(Servo.class, "flowerDescorer");
        blocker.setPosition(UNBLOCKING_POSITION);
    }

    public Command block() {
        return instant(() -> blocker.setPosition(BLOCKING_POSITION));
    }

    public Command unblock() {
        return instant(() -> blocker.setPosition(UNBLOCKING_POSITION));
    }

    @Override
    public void debug() {
        o.telemetry.addData("Blocker - State", blocker.getPosition() == BLOCKING_POSITION ? "Blocking" : "Not blocking");
    }
}
