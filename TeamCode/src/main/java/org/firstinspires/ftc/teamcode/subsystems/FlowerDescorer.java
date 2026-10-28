package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;


public class FlowerDescorer extends Subsystem {
    private final Servo left, right;

    private static final double STOWED_POSITION = 0.5;
    private static final double DEPLOYED_POSITION = 0.0;

    public FlowerDescorer(Robot r) {
        super(r);

        left = r.hardwareMap.get(Servo.class, "flowerLeft");
        right = r.hardwareMap.get(Servo.class, "flowerRight");
        right.setDirection(Servo.Direction.REVERSE);
    }

    public Command stow() {
        return instant(() -> {
            left.setPosition(STOWED_POSITION);
            right.setPosition(STOWED_POSITION);
        });
    }

    public Command deploy() {
        return instant(() -> {
            left.setPosition(DEPLOYED_POSITION);
            right.setPosition(DEPLOYED_POSITION);
        });
    }

    public boolean isDeployed() {
        return left.getPosition() == DEPLOYED_POSITION;
    }

    @Override
    public void debug() {
        r.telemetry.addData("Flower Descorer - Position", isDeployed() ? "Deployed" : "Stowed");
    }
}
