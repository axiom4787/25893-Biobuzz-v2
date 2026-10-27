package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.controllers.PIDController;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.hardware.DcMotorCaching;


public class Turret extends Subsystem {
    private final DcMotorCaching turret;

    private double clippedTargetAngleRadians = 0;
    private boolean canAchieveTarget = true;

    private boolean enabled = true;
    private final PIDController turnController = new PIDController(0.01, 0, 0);

    private static final double CCW_LIMIT_RAD = Math.toRadians(100);
    private static final double CW_LIMIT_RAD = Math.toRadians(-100);
    private static final double MOTOR_TICKS_PER_TURRET_ROTATION;

    static {
        double TICKS_PER_MOTOR_ROTATION = 537.7; // 537.7 ticks per full drive motor rotation (312 RPM motor).
        double GEAR_RATIO = 1.0 / 3.0; // Placeholder, Input gear teeth / output gear teeth

        MOTOR_TICKS_PER_TURRET_ROTATION = TICKS_PER_MOTOR_ROTATION / GEAR_RATIO;
        // A target angle of zero should be facing straight backwards
        // Positive should be ccw,
        // Negative should be cw
    }

    public Turret(Robot r) {
        super(r);

        turret = r.hardwareMap.get(DcMotorCaching.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD); // TODO: Verify direction
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zero();
    }

    public void zero() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getClippedTargetAngleRadians() {
        return clippedTargetAngleRadians;
    }

    public boolean canAchieveUnclippedTarget() {
        return true;
    }

    public Command enable() {
        return instant(() -> enabled = true);
    }

    public Command disable() {
        return instant(() -> enabled = false);
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getActualAngleRadians() {
        double currentTicks = turret.getCurrentPosition();
        double rotations = currentTicks / MOTOR_TICKS_PER_TURRET_ROTATION;
        return rotations * Math.PI * 2;
    }

    public double getErrorRadians() {
        return getClippedTargetAngleRadians() - getActualAngleRadians();
    }

    public double getErrorDegrees() {
        return Math.toDegrees(getErrorRadians());
    }

    @Override
    public void periodic() {
//        Pose robotPose = r.drivetrain.getPose();
//        double robotHeading = robotPose.heading();
//        double robotX = robotPose.x();
//        double robotY = robotPose.y();
//
//        // Get turret in field space using the robot's pose and how far behind the center of the robot the turret is
//        double turretOffset = 3.0; // 3 inches behind center of robot
//        double turretX = robotX - turretOffset * Math.cos(robotHeading);
//        double turretY = robotY - turretOffset * Math.sin(robotHeading);
//
//        // Get the heading towards the goal using trig
//        Pose goalPose = r.drivetrain.getGoalPos();
//        double diffX = goalPose.x() - turretX;
//        double diffY = goalPose.y() - turretY;
//        double headingTowardsGoal = Math.atan2(diffY, diffX);

        // Clip target to the maximum range of the turret
        double unclippedTargetAngleRadians = r.sotm.getTurretAngleRad();
        double clipped = Range.clip(unclippedTargetAngleRadians, CW_LIMIT_RAD, CCW_LIMIT_RAD);
        canAchieveTarget = unclippedTargetAngleRadians == clipped;
        clippedTargetAngleRadians = clipped;
        double power = turnController.calculate(getClippedTargetAngleRadians(), getErrorRadians());

        if (Math.abs(power) < 0.01) power = 0;

        if (enabled) {
            turret.setPower(power);
        } else {
            turret.setPower(0);
        }
    }

    @Override
    public void debug() {
        double targetAngleDeg = Math.toDegrees(clippedTargetAngleRadians);
        double actualAngleDeg = Math.toDegrees(getActualAngleRadians());
        double angleErrorDeg = Math.toDegrees(getErrorRadians());

        r.telemetry.addLine("Turret:");
        r.telemetry.addData("- Angle", "Actual=%3.2f°, Target=%3.2f°, Error=%3.2f°", actualAngleDeg, targetAngleDeg, angleErrorDeg);
        r.telemetry.addData("- Turret power", turret.getPower());
        r.telemetry.addData("- Is enabled", enabled ? "Yes" : "No");
    }
}
