package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

public abstract class Subsystem {
    protected final Robot r;

    /**
     * Subsystem constructor
     * @param r Robot for the subsystem to use
     */
    public Subsystem(Robot r) {
        this.r = r;
    }

    /**
     * Called to request debug information. Log data to telemetry here.
     * Ex: r.telemetry.addLine("Debug log");
     */
    public void debug() {}

    /**
     * Command to run infinitely during the opmode main loop.
     * Usually, return [Commands.]infinite(() -> { ... });
     */
    public Command periodicCommand() { return null; }
}
