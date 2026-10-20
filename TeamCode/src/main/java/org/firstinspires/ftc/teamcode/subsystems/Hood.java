package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.ServoCaching;


public class Hood extends Subsystem {
    private final ServoCaching hood;

    private boolean enabled = true;

    public Hood(Robot r) {
        super(r);

        hood = r.hardwareMap.get(ServoCaching.class, "hood");
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void periodic() {
        double distToGoal = r.drivetrain.getDistanceToGoal();
        // TODO: Do math to determine hood position to set
    }
}
