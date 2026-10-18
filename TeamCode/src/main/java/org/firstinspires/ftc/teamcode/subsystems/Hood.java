package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.ServoCaching;

import static com.pedropathing.ivy.commands.Commands.infinite;


public class Hood extends Subsystem {
    private final ServoCaching hood;

    private boolean enabled = true;
    private double distToGoal = 0;
    private double lastSetPosition = Double.NaN;

    public Hood(OpMode o) {
        super(o);

        hood = o.hardwareMap.get(ServoCaching.class, "hood");
    }

    public void setDistToGoal(double distToGoal) {
        this.distToGoal = distToGoal;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public Command periodicCommand() {
        return infinite(() -> {
            // TODO: Do math to determine hood position to set
        });
    }
}
