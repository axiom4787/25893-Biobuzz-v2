package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import static com.pedropathing.ivy.commands.Commands.infinite;


import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Lights extends Subsystem {
    private final GoBildaPrismDriver lights;

    private State state;
    private enum State {
        AIM_BAR_FULL(true, 10),
        AIM_BAR_8(true, 8),
        AIM_BAR_6(true, 6),
        AIM_BAR_4(true, 4),
        AIM_BAR_2(true, 2),
        AIM_BAR_EMPTY(true, 0),
        OUTSIDE_LAUNCH_ZONE(false, -1),
        OUTSIDE_TURRET_RANGE(false, -1),
        SHOOTING(false, -1),
        OFF(false, -1);

        State(boolean isAimBar, int barWidth) {
            this.isAimBar = isAimBar;
            this.barWidth = barWidth;
        }

        private final boolean isAimBar;
        private final int barWidth;
    }

    private static final PrismAnimations.Solid RED_BACKGROUND = new PrismAnimations.Solid(Color.RED);

    public Lights(Robot r) {
        super(r);

        lights = r.hardwareMap.get(GoBildaPrismDriver.class, "lights");
        lights.setStripLength(20);
        state = State.OFF;
    }

    @Override
    public Command periodicCommand() {
        return infinite(() -> {
            State lastState = state;

            double turretErrDeg = Math.toDegrees(Math.abs(r.turret.getErrorRadians()));
            if (!r.drivetrain.isInScoringZone()) {
                state = State.OUTSIDE_LAUNCH_ZONE;
            } else if (!r.blocker.isBlocking()) {
                state = State.SHOOTING;
            } else if (!r.turret.canAchieveUnclippedTarget()) {
                state = State.OUTSIDE_TURRET_RANGE;
            } else if (turretErrDeg <= 2) {
                state = State.AIM_BAR_FULL;
            } else if (turretErrDeg <= 5) {
                state = State.AIM_BAR_8;
            } else if (turretErrDeg <= 8) {
                state = State.AIM_BAR_6;
            } else if (turretErrDeg <= 12) {
                state = State.AIM_BAR_4;
            } else {
                state = State.AIM_BAR_EMPTY;
            }

            if (state == lastState) return;

            // TODO: Actually update the light strip
        });
    }

    private PrismAnimations.Solid greenBar(int width) {
        PrismAnimations.Solid greenBar = new PrismAnimations.Solid(Color.GREEN);

        int startIndex = (10 - width) / 2;
        int stopIndex = startIndex + width - 1;

        greenBar.setIndexes(startIndex, stopIndex);

        return greenBar;
    }
}
