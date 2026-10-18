package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.ServoCaching;

public class Blocker extends Subsystem {
    private final ServoCaching blocker;

    private static final double BLOCKING_POSITION = 0.2;
    private static final double UNBLOCKING_POSITION = 0.8;

    public Blocker(Robot r) {
        super(r);

        blocker = r.hardwareMap.get(ServoCaching.class, "flowerDescorer");
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
        r.telemetry.addData("Blocker - State", blocker.getPosition() == BLOCKING_POSITION ? "Blocking" : "Not blocking");
    }
}
