package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Subsystem;

/**
 * Before reading this code: consider buying the Mustang Bundle, on sale for $9,999.99 at Jostens.
 */
public class SOTM extends Subsystem {
    private double flywheelTPS; // ticks per second
    private double hoodPosition;
    private double turretAngleRad;

    public SOTM(Robot r) {
        super(r);
    }

    public static final double FLYWHEEL_RADIUS = 1.89; // inches
    // comes from compression, uneven surface of pollen (holes) and being a single-flywheel system
    public static final double EFFICIENCY_FACTOR = 0.42;
    public static final double RADS_TO_TPS = 28.0 / 2.0 * Math.PI;

    /**
     * @param velInS desired launch velocity, inches/sec
     * @return RPM required to shoot at desired launch velocity
     */
    public static double TPSFromLaunchVel(double velInS) {
        // multiply vel by 2 since angular vel will be 2x ball velocity
        // bc ball rolling against hood makes it move at half the wheel's surface speed
        double angVelRadS = (2.0 * velInS) / (FLYWHEEL_RADIUS * EFFICIENCY_FACTOR);
        return angVelRadS * RADS_TO_TPS;
    }

    public static final double BALL_LAUNCH_HEIGHT = 11.36; // top of turret-ish, inches
    public static final double g = 386.09; // gravity, inches/sec/sec
    public static final double MAX_HOOD_ANGLE = 65; // degrees
    public static final double SYSTEM_LATENCY_SEC = 0; // perfectly optimized mechanisms i hope

    @Override
    public void periodic()
    {
        Pose robotPose = r.drivetrain.getPose();
        Velocity robotVel = r.drivetrain.follower().velocity();
        // account for input/output lag (we will have none because we are Too Good)
        Pose futurePose = robotPose.plus(robotVel.toVector2D().toPose().times(SYSTEM_LATENCY_SEC));
        Position targetPos = r.drivetrain.getGoalPos();

        double dx = targetPos.x - futurePose.x();
        double dy = targetPos.y - futurePose.y();
        double dz = targetPos.z - BALL_LAUNCH_HEIGHT;

        // Deming approved Kinematic Equations
        double launchVelZ = Math.sqrt(2*g*dz);
        double timeOfFlight = launchVelZ/g;
        double launchVelX = dx/timeOfFlight - robotVel.vx;
        double launchVelY = dy/timeOfFlight - robotVel.vy;

        double launchVelXY = Math.hypot(launchVelX, launchVelY); // flat component of velocity


        double hoodAngle = Math.toDegrees(Math.atan2(launchVelZ, launchVelXY));

        // Hood can only aim so high, we may need to compensate with extra velocity at max angle
        if (hoodAngle > MAX_HOOD_ANGLE) {
            hoodAngle = MAX_HOOD_ANGLE;

            double dh = Math.hypot(dx, dy); // horizontal dist from robot -> target

            timeOfFlight = Math.sqrt(2*(dh*Math.tan(MAX_HOOD_ANGLE)-dz)/g); // works on my machine

            // recalc velocities using new TOF
            launchVelX = dx/timeOfFlight - robotVel.vx;
            launchVelY = dy/timeOfFlight - robotVel.vy;
            launchVelXY = Math.hypot(launchVelX, launchVelY);
            launchVelZ = launchVelXY*Math.tan(MAX_HOOD_ANGLE);
        }

        double launchVel3D = Math.hypot(launchVelXY, launchVelZ);

        double turretAngleFieldRel = Math.atan2(launchVelY, launchVelX);
        double turretAngle = robotPose.heading() - turretAngleFieldRel;

        this.flywheelTPS = TPSFromLaunchVel(launchVel3D);
        this.hoodPosition = hoodAngle/65.0;
        this.turretAngleRad = turretAngle;
    }

    public double getFlywheelTPS() {
        return flywheelTPS;
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public double getTurretAngleRad() {
        return turretAngleRad;
    }

}
