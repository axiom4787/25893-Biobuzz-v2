package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Before reading this code: consider buying the Mustang Bundle, on sale for $9,999.99 at Jostens.
 */
public class ScoringUtils {
    /**
     * advanced targeting data sponsored by Lockheed Martin
     */
    public class LaunchParams {
        public double flywheelRPM;
        public double hoodAngleDeg;
        public double turretAngleDeg;

        /**
         * @param f flywheel rpm
         * @param h hood launch angle degrees, 0=horizontal (probably impossible icl)
         * @param t turret angle degrees, 0=opposite intake
         */
        public LaunchParams(double f, double h, double t) {
            flywheelRPM = f;
            hoodAngleDeg = h;
            turretAngleDeg = t;
        }
    }
    public static final double FLYWHEEL_RADIUS = 1.89; // inches

    // comes from compression, uneven surface of pollen (holes) and being a single-flywheel system
    public static final double EFFICIENCY_FACTOR = 0.42;

    public static final double RADS_TO_RPM = 60.0 / 2.0 * Math.PI;

    /**
     * @param velInS desired launch velocity, inches/sec
     * @return RPM required to shoot at desired launch velocity
     */
    public static double RPMFromLaunchVel(double velInS) {
        // multiply vel by 2 since angular vel will be 2x ball velocity
        // bc ball rolling against hood makes it move at half the wheel's surface speed
        double angVelRadS = (2.0 * velInS) / (FLYWHEEL_RADIUS * EFFICIENCY_FACTOR);
        return angVelRadS * RADS_TO_RPM;
    }

    public static final double BALL_LAUNCH_HEIGHT = 11.36; // top of turret-ish, inches
    public static final double g = 386.09; // gravity, inches/sec/sec
    public static final double MAX_HOOD_ANGLE = 65; // degrees
    public static final double SYSTEM_LATENCY_SEC = 0; // perfectly optimized mechanisms i hope

    public LaunchParams SOTM(Pose robotPose, Pose robotVel, Position targetPos)
    {
        // account for input/output lag (we will have none because we are Too Good)
        Pose futurePose = robotPose.plus(robotVel.times(SYSTEM_LATENCY_SEC));

        double dx = targetPos.x - futurePose.x();
        double dy = targetPos.y - futurePose.y();
        double dz = targetPos.z - BALL_LAUNCH_HEIGHT;

        // Deming approved Kinematic Equations
        double launchVelZ = Math.sqrt(2*g*dz);
        double timeOfFlight = launchVelZ/g;
        double launchVelX = dx/timeOfFlight - robotVel.x();
        double launchVelY = dy/timeOfFlight - robotVel.y();

        double launchVelXY = Math.hypot(launchVelX, launchVelY); // flat component of velocity


        double hoodAngle = Math.toDegrees(Math.atan2(launchVelZ, launchVelXY));

        // Hood can only aim so high, we may need to compensate with extra velocity at max angle
        if (hoodAngle > MAX_HOOD_ANGLE) {
            hoodAngle = MAX_HOOD_ANGLE;

            double dh = Math.hypot(dx, dy); // horizontal dist from robot -> target

            timeOfFlight = Math.sqrt(2*(dh*Math.tan(MAX_HOOD_ANGLE)-dz)/g); // works on my machine

            // recalc velocities using new TOF
            launchVelX = dx/timeOfFlight - robotVel.x();
            launchVelY = dy/timeOfFlight - robotVel.y();
            launchVelXY = Math.hypot(launchVelX, launchVelY);
            launchVelZ = launchVelXY*Math.tan(MAX_HOOD_ANGLE);
        }

        double launchVel3D = Math.hypot(launchVelXY, launchVelZ);

        double turretAngleFieldRel = Math.toDegrees(Math.atan2(launchVelY, launchVelX));
        double turretAngle = robotPose.heading() - turretAngleFieldRel;

        double flywheelRPM = RPMFromLaunchVel(launchVel3D);

        return new LaunchParams(flywheelRPM, hoodAngle, turretAngle); // advanced thermonuclear
    }
}
