package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Subsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;


/**
 * Subsystem for managing Limelight vision processing with HiveVision integration.
 * Calculates field-relative positions of detected game pieces (balls/pollen)
 * using 3D camera-to-robot transformation and the robot's pose from PedroPathing.
 */
public class Vision extends Subsystem {
    private final Limelight3A limelight;

    // Camera mounting extrinsics (relative to robot center on ground)
    private double cameraXOffset = Constants.LIMELIGHT_X_OFFSET; // inches forward from robot center
    private double cameraYOffset = Constants.LIMELIGHT_Y_OFFSET; // inches left from robot center
    private double cameraZOffset = Constants.LIMELIGHT_Z_OFFSET; // inches above ground to lens
    private double cameraPitchDegrees = Constants.LIMELIGHT_PITCH_DEG; // pitch angle (deg, negative = tilted down)
    private double cameraYawDegrees = Constants.LIMELIGHT_YAW_DEG;     // yaw angle offset relative to robot forward (deg)

    // Ball/Game piece physical properties
    private double ballRadius = Constants.BALL_RADIUS; // height of ball center above ground (inches)

    // Latest detected balls transformed to field coordinates
    private final List<DetectedBall> detectedBalls = new ArrayList<>();
    private LLResult latestLLResult = null;

    /**
     * Represents a detected ball/game piece with 2D/3D camera, robot-relative, and field-relative coordinates.
     */
    public static class DetectedBall {
        public final Pose fieldPose;          // Field X, Y (inches) and heading
        public final double robotX;           // Robot-relative X (inches forward from robot center)
        public final double robotY;           // Robot-relative Y (inches left from robot center)
        public final double cameraX;          // Camera-relative forward distance (inches)
        public final double cameraY;          // Camera-relative lateral distance (inches)
        public final double tx;               // Target horizontal offset angle from crosshair (deg)
        public final double ty;               // Target vertical offset angle from crosshair (deg)
        public final String className;        // Object class/label ("ball", "pollen", "red", "blue", "yellow", etc.)
        public final double confidence;       // Confidence score
        public final long timestampMs;        // System timestamp of detection

        public DetectedBall(Pose fieldPose, double robotX, double robotY,
                            double cameraX, double cameraY,
                            double tx, double ty, String className,
                            double confidence, long timestampMs) {
            this.fieldPose = fieldPose;
            this.robotX = robotX;
            this.robotY = robotY;
            this.cameraX = cameraX;
            this.cameraY = cameraY;
            this.tx = tx;
            this.ty = ty;
            this.className = className;
            this.confidence = confidence;
            this.timestampMs = timestampMs;
        }

        public double getDistanceToRobot() {
            return Math.hypot(robotX, robotY);
        }

        public double getDistanceToFieldPose(Pose targetPose) {
            double dx = fieldPose.x() - targetPose.x();
            double dy = fieldPose.y() - targetPose.y();
            return Math.hypot(dx, dy);
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "Ball[%s] Field(X=%.2f, Y=%.2f) Bot(X=%.2f, Y=%.2f) Conf=%.2f",
                    className, fieldPose.x(), fieldPose.y(), robotX, robotY, confidence);
        }
    }

    public Vision(Robot r) {
        super(r);

        limelight = r.hardwareMap.get(Limelight3A.class, "limelight");
        start();
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    public void setCameraExtrinsics(double xOffset, double yOffset, double zOffset,
                                   double pitchDegrees, double yawDegrees) {
        this.cameraXOffset = xOffset;
        this.cameraYOffset = yOffset;
        this.cameraZOffset = zOffset;
        this.cameraPitchDegrees = pitchDegrees;
        this.cameraYawDegrees = yawDegrees;
    }

    public void setBallRadius(double radiusInches) {
        this.ballRadius = radiusInches;
    }

    @Override
    public void periodic() {
        latestLLResult = limelight.getLatestResult();
        detectedBalls.clear();

        if (latestLLResult == null || !latestLLResult.isValid()) {
            return;
        }

        Pose robotPose = r.drivetrain.getPose();
        long now = System.currentTimeMillis();

        // 1. Process Detector Results (Neural Network / AI pipeline from HiveVision)
        List<LLResultTypes.DetectorResult> detectors = latestLLResult.getDetectorResults();
        if (detectors != null && !detectors.isEmpty()) {
            for (LLResultTypes.DetectorResult dr : detectors) {
                DetectedBall ball = processTarget(
                        dr.getTargetXDegrees(),
                        dr.getTargetYDegrees(),
                        dr.getClassName(),
                        dr.getConfidence(),
                        robotPose,
                        now
                );
                if (ball != null) {
                    detectedBalls.add(ball);
                }
            }
        }
        // 2. Process Color Pipeline Results (if using color thresholding)
        else {
            List<LLResultTypes.ColorResult> colors = latestLLResult.getColorResults();
            if (colors != null && !colors.isEmpty()) {
                for (LLResultTypes.ColorResult cr : colors) {
                    DetectedBall ball = processTarget(
                            cr.getTargetXDegrees(),
                            cr.getTargetYDegrees(),
                            "ball",
                            1.0,
                            robotPose,
                            now
                    );
                    if (ball != null) {
                        detectedBalls.add(ball);
                    }
                }
            }
            // 3. Fallback to main target if valid tx/ty present
            else if (latestLLResult.getTa() > 0) {
                DetectedBall ball = processTarget(
                        latestLLResult.getTx(),
                        latestLLResult.getTy(),
                        "ball",
                        1.0,
                        robotPose,
                        now
                );
                if (ball != null) {
                    detectedBalls.add(ball);
                }
            }
        }
    }

    /**
     * Converts 2D Limelight crosshair angles (tx, ty in degrees) to field-relative Pose.
     * Uses camera height, pitch, yaw, ball radius, and current robot pose.
     */
    public DetectedBall processTarget(double tx, double ty, String className, double confidence,
                                      Pose robotPose, long timestampMs) {
        // Delta height between camera lens and center of ball
        double deltaZ = cameraZOffset - ballRadius;
        if (deltaZ <= 0) return null; // Invalid camera height

        // Total vertical angle relative to horizon (in radians)
        // Limelight ty is positive upwards, negative downwards.
        // cameraPitchDegrees is pitch angle relative to horizon (negative when tilted down).
        double pitchRad = Math.toRadians(cameraPitchDegrees);
        double tyRad = Math.toRadians(ty);
        double totalElevationAngleRad = pitchRad + tyRad;

        // Angle below horizontal to floor plane = -totalElevationAngleRad
        double angleBelowHorizontal = -totalElevationAngleRad;
        if (angleBelowHorizontal <= 0.01) {
            // Target is at or above horizon, ray will not intersect floor plane
            return null;
        }

        // Forward distance along camera's ground projection
        double cameraForwardDist = deltaZ / Math.tan(angleBelowHorizontal);

        // Lateral distance relative to camera forward axis
        // Limelight tx is positive to the right. In left-positive Y frame, lateral distance = -forward * tan(tx)
        double txRad = Math.toRadians(tx);
        double cameraLateralDist = -cameraForwardDist * Math.tan(txRad);

        // Convert camera-relative (forward, lateral) to robot-relative (X, Y)
        double yawRad = Math.toRadians(cameraYawDegrees);
        double robotX = cameraXOffset + (cameraForwardDist * Math.cos(yawRad) - cameraLateralDist * Math.sin(yawRad));
        double robotY = cameraYOffset + (cameraForwardDist * Math.sin(yawRad) + cameraLateralDist * Math.cos(yawRad));

        // Convert robot-relative (robotX, robotY) to field-relative (fieldX, fieldY)
        // robotPose.heading() is in radians in PedroPathing
        double robotHeadingRad = robotPose.heading();
        double fieldX = robotPose.x() + (robotX * Math.cos(robotHeadingRad) - robotY * Math.sin(robotHeadingRad));
        double fieldY = robotPose.y() + (robotX * Math.sin(robotHeadingRad) + robotY * Math.cos(robotHeadingRad));

        Pose fieldPose = new Pose(fieldX, fieldY, 0);

        return new DetectedBall(
                fieldPose,
                robotX,
                robotY,
                cameraForwardDist,
                cameraLateralDist,
                tx,
                ty,
                className,
                confidence,
                timestampMs
        );
    }

    /**
     * Returns all currently detected balls/game pieces with field-relative poses.
     */
    public List<DetectedBall> getDetectedBalls() {
        return new ArrayList<>(detectedBalls);
    }

    /**
     * Returns detected balls filtered by class name or color keyword (e.g. "yellow_pollen", "yellow", "red", "blue").
     */
    public List<DetectedBall> getDetectedBalls(String classNameOrColor) {
        List<DetectedBall> filtered = new ArrayList<>();
        if (classNameOrColor == null) return filtered;
        String query = classNameOrColor.toLowerCase(Locale.US);
        for (DetectedBall ball : detectedBalls) {
            if (ball.className != null) {
                String name = ball.className.toLowerCase(Locale.US);
                if (name.equalsIgnoreCase(query) || name.contains(query)) {
                    filtered.add(ball);
                }
            }
        }
        return filtered;
    }

    /**
     * Returns detected yellow pollen game elements.
     */
    public List<DetectedBall> getYellowPollen() {
        return getDetectedBalls("yellow");
    }

    /**
     * Returns detected red nectar game elements.
     */
    public List<DetectedBall> getRedNectar() {
        return getDetectedBalls("red");
    }

    /**
     * Returns detected blue nectar game elements.
     */
    public List<DetectedBall> getBlueNectar() {
        return getDetectedBalls("blue");
    }

    /**
     * Returns the closest detected ball to the robot.
     */
    public DetectedBall getClosestBall() {
        DetectedBall closest = null;
        double minDist = Double.MAX_VALUE;
        for (DetectedBall ball : detectedBalls) {
            double dist = ball.getDistanceToRobot();
            if (dist < minDist) {
                minDist = dist;
                closest = ball;
            }
        }
        return closest;
    }

    /**
     * Returns the closest detected ball to a specific field position.
     */
    public DetectedBall getClosestBallTo(Pose targetFieldPose) {
        DetectedBall closest = null;
        double minDist = Double.MAX_VALUE;
        for (DetectedBall ball : detectedBalls) {
            double dist = ball.getDistanceToFieldPose(targetFieldPose);
            if (dist < minDist) {
                minDist = dist;
                closest = ball;
            }
        }
        return closest;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public LLResult getLatestLLResult() {
        return latestLLResult;
    }

    @Override
    public void debug() {
        r.telemetry.addLine("Vision (Limelight / HiveVision):");
        if (latestLLResult != null && latestLLResult.isValid()) {
            r.telemetry.addData("- Target count", detectedBalls.size());
            r.telemetry.addData("- FPS", (int) limelight.getStatus().getFps());
            r.telemetry.addData("- Latency (ms)", latestLLResult.getCaptureLatency() + latestLLResult.getTargetingLatency());
            for (int i = 0; i < Math.min(3, detectedBalls.size()); i++) {
                DetectedBall b = detectedBalls.get(i);
                r.telemetry.addData(String.format(Locale.US, "- Ball %d [%s]", i, b.className),
                        "Field(%.1f, %.1f) Bot(%.1f, %.1f) Tx=%.1f Ty=%.1f",
                        b.fieldPose.x(), b.fieldPose.y(), b.robotX, b.robotY, b.tx, b.ty);
            }
        } else {
            r.telemetry.addData("- Status", "No valid target");
        }
    }
}
