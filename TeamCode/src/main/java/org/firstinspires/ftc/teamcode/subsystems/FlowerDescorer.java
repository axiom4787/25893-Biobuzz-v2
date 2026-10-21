package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.ServoCaching;


public class FlowerDescorer extends Subsystem {
    private final ServoCaching arm;

    private static final double STOWED_POSITION = 0.2; // Placeholder
    private static final double DEPLOYED_POSITION = 0.8; // Placeholder

    public FlowerDescorer(Robot r) {
        super(r);

        arm = r.hardwareMap.get(ServoCaching.class, "flowerDescorer");
        arm.setPosition(STOWED_POSITION);
    }

    public Command stow() {
        return instant(() -> arm.setPosition(STOWED_POSITION));
    }

    public Command deploy() {
        return instant(() -> arm.setPosition(DEPLOYED_POSITION));
    }

    public boolean isDeployed() {
        return arm.getPosition() == DEPLOYED_POSITION;
    }

    @Override
    public void debug() {
        r.telemetry.addData("Flower Descorer - Position", isDeployed() ? "Deployed" : "Stowed");
    }
}
