package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.controllers.PIDController;
import com.pedropathing.ivy.Command;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.DcMotorCaching;


public class Turret extends Subsystem {
    private final DcMotorCaching turret;

    private double targetAngleRadians = 0;
    private boolean enabled = true;
    private final PIDController turnController = new PIDController(0.01, 0, 0);

    private static final double CCW_LIMIT_RAD = Math.toRadians(60);
    private static final double CW_LIMIT_RAD = Math.toRadians(-60);
    private static final double MOTOR_TICKS_PER_TURRET_ROTATION;
    static {
        double TICKS_PER_MOTOR_ROTATION = 537.7; // 537.7 ticks per full drive motor rotation.
        double GEAR_RATIO = 1.0 / 3.0; // Input gear teeth / output gear teeth

        MOTOR_TICKS_PER_TURRET_ROTATION = TICKS_PER_MOTOR_ROTATION / GEAR_RATIO;
        // A target angle of zero should be facing straight backwards
        // Positive should be ccw,
        // Negative should be cw
    }

    public Turret(Robot r) {
        super(r);

        turret = r.hardwareMap.get(DcMotorCaching.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zero();
    }

    public void zero() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    };

    public void setTargetAngleRadians(double angleRadians) {
        double normalized = Angle.normalizeSigned(angleRadians);
        if (Math.abs(normalized) > Math.toRadians(150)) {
            return; // Don't change the turret target if it would be in the 60 degrees behind zero
        }
        targetAngleRadians = Range.clip(normalized, CW_LIMIT_RAD, CCW_LIMIT_RAD);
    }

    public double getTargetAngleRadians() {
        return targetAngleRadians;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean getEnabled() {
        return enabled;
    }

    public double getActualAngleRadians() {
        double currentTicks = turret.getCurrentPosition();
        double rotations = currentTicks / MOTOR_TICKS_PER_TURRET_ROTATION;
        return rotations * Math.PI * 2;
    }

    public double getErrorRadians() {
        return getActualAngleRadians() - getTargetAngleRadians();
    }

    @Override
    public Command periodicCommand() {
        return infinite(() -> {
            double power = turnController.calculate(getTargetAngleRadians(), getActualAngleRadians());

            if (enabled) {
                turret.setPower(power);
            } else {
                turret.setPower(0);
            }
        });
    }

    @Override
    public void debug() {
        double targetAngleDeg = Math.toDegrees(targetAngleRadians);
        double actualAngleDeg = Math.toDegrees(getActualAngleRadians());
        double angleErrorDeg = Math.toDegrees(getErrorRadians());

        r.telemetry.addLine("Turret:");
        r.telemetry.addData("- Angle", "Actual=%3.2f°, Target=%3.2f°, Error=%3.2f°", actualAngleDeg, targetAngleDeg, angleErrorDeg);
        r.telemetry.addData("- Turret power", turret.getPower());
        r.telemetry.addData("- Is enabled", enabled ? "Yes" : "No");
    }
}
