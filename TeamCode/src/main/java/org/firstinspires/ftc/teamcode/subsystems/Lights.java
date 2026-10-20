package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Lights extends Subsystem {
    private final GoBildaPrismDriver lights;

    private State state;
    private enum State {
        AIM_BAR_FULL(10),
        AIM_BAR_8(8),
        AIM_BAR_6(6),
        AIM_BAR_4(4),
        AIM_BAR_2(2),
        AIM_BAR_EMPTY( 0),
        OUTSIDE_LAUNCH_ZONE(-1),
        OUTSIDE_TURRET_RANGE(-1),
        SHOOTING(-1),
        OFF(-1);

        State(int aimBarWidth) {
            this.aimBarWidth = aimBarWidth;
        }

        final int aimBarWidth;

        static int NO_BAR_SENTINEL = -1;
    }

    private static final PrismAnimations.Solid RED_BACKGROUND = new PrismAnimations.Solid(Color.RED);

    public Lights(Robot r) {
        super(r);

        lights = r.hardwareMap.get(GoBildaPrismDriver.class, "lights");
        lights.setStripLength(20);
        state = State.OFF;
    }

    private PrismAnimations.Solid greenBar(int width) {
        PrismAnimations.Solid greenBar = new PrismAnimations.Solid(Color.GREEN);

        int startIndex = (10 - width) / 2;
        int stopIndex = startIndex + width - 1;

        greenBar.setIndexes(startIndex, stopIndex);

        return greenBar;
    }

    @Override
    public void periodic() {
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
        } else if (turretErrDeg <= 20) {
            state = State.AIM_BAR_2;
        } else {
            state = State.AIM_BAR_EMPTY;
        }

        if (state == lastState) return;

        // TODO: Actually update the light strip

        lights.clearAllAnimations();

        if (state.aimBarWidth != State.NO_BAR_SENTINEL) {
            lights.insertAnimation(LayerHeight.LAYER_0, RED_BACKGROUND);
            lights.insertAnimation(LayerHeight.LAYER_1, greenBar(state.aimBarWidth));
        } else if (state == State.OUTSIDE_TURRET_RANGE) {
            PrismAnimations.Blink purpleBlink = new PrismAnimations.Blink(Color.PURPLE);
            lights.insertAnimation(LayerHeight.LAYER_0, purpleBlink);
        } else if (state == State.OUTSIDE_LAUNCH_ZONE) {
            PrismAnimations.Solid solidYellow = new PrismAnimations.Solid(Color.YELLOW);
            lights.insertAnimation(LayerHeight.LAYER_0, solidYellow);
        } else if (state == State.SHOOTING) {
            PrismAnimations.Rainbow rainbow = new PrismAnimations.Rainbow();
            lights.insertAnimation(LayerHeight.LAYER_0, rainbow);
        }

        lights.updateAllAnimations();
    }

    @Override
    public void debug() {
        r.telemetry.addData("Lights - State", state.name());
    }
}
