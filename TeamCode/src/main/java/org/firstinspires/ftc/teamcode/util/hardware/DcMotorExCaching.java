package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.Range;

public class DcMotorExCaching extends DcMotorImplEx {
    /// Setting power will be ignored if the new power is within this threshold of the last set power
    protected double cachingThreshold = 0.02;
    protected double lastSetPower = 0.0;

    public DcMotorExCaching(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
    }

    @Override
    protected void internalSetPower(double power) {
        power = Range.clip(power, -1, 1);

        if (Math.abs(power - lastSetPower) > cachingThreshold
                || power == 0 && lastSetPower != 0
                || power == 1 && lastSetPower != 1
                || power == -1 && lastSetPower != -1
        ) {
            // Added write caching: Don't set the same power again if it's within the caching threshold of the last power
            super.internalSetPower(power);
            lastSetPower = power;
        }
    }

    public double getCachingThreshold() {
        return cachingThreshold;
    }

    public void setCachingThreshold(double cachingThreshold) {
        this.cachingThreshold = cachingThreshold;
    }
}
