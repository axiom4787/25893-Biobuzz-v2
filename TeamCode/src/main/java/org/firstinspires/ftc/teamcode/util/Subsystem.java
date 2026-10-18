package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Subsystem {
    protected final OpMode o;

    public Subsystem(OpMode o) {
        this.o = o;
    }

    public void debug() {}

    public Command periodicCommand() { return null; }
}
