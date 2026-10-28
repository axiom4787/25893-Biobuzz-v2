package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;


public class Blocker extends Subsystem {
    private final Servo blocker;

    private static final double BLOCKING_POSITION = 0.2; // Placeholder
    private static final double UNBLOCKING_POSITION = 0.8; // Placeholder

    public Blocker(Robot r) {
        super(r);

        blocker = r.hardwareMap.get(Servo.class, "flowerDescorer");
        blocker.setPosition(UNBLOCKING_POSITION);
    }

    public Command block() {
        return instant(() -> blocker.setPosition(BLOCKING_POSITION));
    }

    public Command unblock() {
        return instant(() -> blocker.setPosition(UNBLOCKING_POSITION));
    }

    public boolean isBlocking() {
        return blocker.getPosition() == BLOCKING_POSITION;
    }

    @Override
    public void debug() {
        r.telemetry.addData("Blocker - State", isBlocking() ? "Blocking" : "Not blocking");
    }
}
