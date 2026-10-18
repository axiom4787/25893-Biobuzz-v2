package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.commands.Commands.*;

import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.ServoCaching;

public class FlowerDescorer extends Subsystem {
    private final ServoCaching arm;

    private static final double STOWED_POSITION = 0.2;
    private static final double DEPLOYED_POSITION = 0.8;

    public FlowerDescorer(OpMode o) {
        super(o);

        arm = o.hardwareMap.get(ServoCaching.class, "flowerDescorer");
        arm.setPosition(STOWED_POSITION);
    }

    public Command stow() {
        return instant(() -> arm.setPosition(STOWED_POSITION));
    }

    public Command deploy() {
        return instant(() -> arm.setPosition(DEPLOYED_POSITION));
    }

    @Override
    public void debug() {
        o.telemetry.addData("Flower Descorer - Position", arm.getPosition() == STOWED_POSITION ? "Stowed" : "Deployed");
    }
}
