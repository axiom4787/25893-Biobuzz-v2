package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class DcMotorCaching {
    /// Setting power will be ignored if the new power is within this threshold of the last set power
    private double cachingThreshold = 0.02;
    private double lastSetPower = 0.0;

    private final DcMotor motor;

    public DcMotorCaching(HardwareMap hardwareMap, String id) {
        motor = hardwareMap.get(DcMotorEx.class, id);
    }

    public double getCachingThreshold() {
        return cachingThreshold;
    }

    public void setCachingThreshold(double cachingThreshold) {
        this.cachingThreshold = cachingThreshold;
    }
}
