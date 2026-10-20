package org.firstinspires.ftc.teamcode.util;

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
     * Code to run infinitely during the opmode main loop.
     */
    public void periodic() {}
}
