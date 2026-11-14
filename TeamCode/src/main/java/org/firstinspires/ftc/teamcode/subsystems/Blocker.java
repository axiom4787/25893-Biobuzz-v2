package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;


public class Blocker extends Subsystem {
    private final CachingServo blocker;

    private static final double BLOCKING_POSITION = 0.25;
    private static final double UNBLOCKING_POSITION = 0.5;

    private boolean isBlocking = false;

    public Blocker(Robot r) {
        super(r);

        blocker = new CachingServo(r.hardwareMap.get(Servo.class, "blocker"));

        block().schedule();
    }

    public Command block() {
        return instant(() -> {
            blocker.setPosition(BLOCKING_POSITION);
            isBlocking = true;
        });
    }

    public Command unblock() {
        return instant(() -> {
            blocker.setPosition(UNBLOCKING_POSITION);
            isBlocking = false;
        });
    }

    public boolean isBlocking() {
        return isBlocking;
    }

    @Override
    public void debug() {
        r.telemetry.addData("Blocker - State", isBlocking() ? "Blocking" : "Not blocking");
    }
}
