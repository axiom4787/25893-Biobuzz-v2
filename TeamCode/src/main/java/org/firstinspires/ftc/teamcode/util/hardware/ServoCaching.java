package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.Range;

public class ServoCaching extends ServoImpl {
    /// Setting position will be ignored if the new position is within this threshold of the last set position
    protected double cachingThreshold = 0.02;
    protected double lastSetPosition = 0.0;

    public ServoCaching(ServoController controller, int portNumber) {
        super(controller, portNumber);
    }

    @Override
    protected void internalSetPosition(double position) {
        position = Range.clip(position, 0, 1);

        if (Math.abs(position - lastSetPosition) > cachingThreshold
                || position == 0 && lastSetPosition != 0
                || position == 1 && lastSetPosition != 1
        ) {
            // Added write caching: Don't set the same position again if it's within the caching threshold of the last position
            super.internalSetPosition(position);
            lastSetPosition = position;
        }
    }

    public double getCachingThreshold() {
        return cachingThreshold;
    }

    public void setCachingThreshold(double cachingThreshold) {
        this.cachingThreshold = cachingThreshold;
    }
}
