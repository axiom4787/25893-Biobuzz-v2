package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;


public class Hood extends Subsystem {
    private final Servo hood;

    private boolean enabled = true;

    private static final double UP_POSITION = 0.8; // Placeholder
    private static final double DOWN_POSITION = 0.2; // Placeholder
    private double percentPos = 0; // scaled, down pos = 0.0, up pos = 1.0

    public Hood(Robot r) {
        super(r);

        hood = r.hardwareMap.get(Servo.class, "hood");
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void periodic() {
        percentPos = r.sotm.getHoodPosition();
        double actualPos = Range.scale(percentPos, 0, 1, DOWN_POSITION, UP_POSITION);
        hood.setPosition(actualPos);
    }

    @Override
    public void debug() {
        r.telemetry.addData("Hood - Position", "%1.2f", percentPos);
    }
}
