package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor;


public class Turret extends Subsystem {
    private final CachingDcMotor turret;

    private double clippedTargetAngleRadians = 0;
    private double unclippedTargetAngleRadians = 0;
    private boolean canAchieveTarget = true;

    private boolean enabled = true;
    private final Controller turnController =
            Controller.pid(0.3, 0, 0)
                    .plus(Controller.staticFeedforward(0.05));

    private static final double CCW_LIMIT_RAD = Math.toRadians(90);
    private static final double CW_LIMIT_RAD = Math.toRadians(-145);
    private static final double MOTOR_TICKS_PER_TURRET_ROTATION;

    static {
        double TICKS_PER_MOTOR_ROTATION = 537.7; // 537.7 ticks per full drive motor rotation (312 RPM motor).
        double GEAR_RATIO = 26.0 / 60.0; // Input gear teeth / output gear teeth

        MOTOR_TICKS_PER_TURRET_ROTATION = TICKS_PER_MOTOR_ROTATION / GEAR_RATIO;
        // A target angle of zero should be facing straight backwards
        // Positive should be ccw,
        // Negative should be cw
    }

    public Turret(Robot r) {
        super(r);

        turret = new CachingDcMotor(r.hardwareMap.get(DcMotor.class, "turret"));
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
        Pose robotPose = r.drivetrain.getPose();
        double robotHeading = robotPose.heading();
        double robotX = robotPose.x();
        double robotY = robotPose.y();

        // Get turret in field space using the robot's pose and how far behind the center of the robot the turret is
        double turretOffset = 0.762; // 3 inches behind center of robot
        double turretX = robotX - turretOffset * Math.cos(robotHeading);
        double turretY = robotY - turretOffset * Math.sin(robotHeading);

        // Get the heading towards the goal using trig
        Position goalPose = r.drivetrain.getGoalPos();
        double diffX = turretX - goalPose.x;
        double diffY = turretY - goalPose.y;
        double headingTowardsGoal = Math.atan2(diffY, diffX);
        unclippedTargetAngleRadians = Angle.normalizeSigned(headingTowardsGoal - robotHeading);

        // Clip target to the maximum range of the turret
//        unclippedTargetAngleRadians = Angle.normalizeSigned(r.sotm.getTurretAngleRad());
        double clipped = Range.clip(unclippedTargetAngleRadians, CW_LIMIT_RAD, CCW_LIMIT_RAD);
        canAchieveTarget = unclippedTargetAngleRadians == clipped;
        clippedTargetAngleRadians = clipped;
        double power = turnController.calculate(getClippedTargetAngleRadians(), getErrorRadians());

        if (enabled && Math.abs(power) > 0.05) {
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
        r.telemetry.addData("- Unclipped", Math.toDegrees(unclippedTargetAngleRadians));
        r.telemetry.addData("- Is enabled", enabled ? "Yes" : "No");
    }
}
