package org.firstinspires.ftc.teamcode.subsystems;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Subsystem;

import java.util.List;


public class LimeLight extends Subsystem {
    private static final boolean ACTIVE = false;
    private static final double ACCEPTABLE_FIDUCIAL_PITCH = 0.0;
    private static final double ACCEPTABLE_FIDUCIAL_PITCH_PM_RANGE = 2.0; // The +- range that a fiducial's yaw must be within at ACCEPTABLE_FIDUCIAL_YAW to be valid
    private static final int AUDI_SIDE_PIPELINE = 0;
    private static final int SCORE_SIDE_PIPELINE = 1;
    private static final double MINIMUM_CONF = 0.7;
    private static final boolean isRedAlliance = true; // TODO: make selectable
    private final Follower follower;
    private final Limelight3A limeLight;

    private LLResult current_result;
    private Pose3D botpose;

    public LimeLight(Robot r, Follower follower) {
        super(r);

        this.follower = follower;
        this.limeLight = r.hardwareMap.get(Limelight3A.class, "limelight");

        LLStatus status = limeLight.getStatus();

        int pipelineIndex = getPipelineByPose();
        if (status.getPipelineIndex() != pipelineIndex) { limeLight.pipelineSwitch(pipelineIndex); }

        limeLight.start();
    }

    public int getPipelineByPose() {
        boolean onRightSide = follower().pose().y() < Constants.HALF_FIELD_WIDTH;

        // right side red alliance and left side blue alliance is audience tags
        // left side red alliance and right side blue alliance is scoring tags
        boolean onAudiSide = onRightSide == isRedAlliance;

        if (onAudiSide) return AUDI_SIDE_PIPELINE;
        return SCORE_SIDE_PIPELINE;
    }

    public Pose getVisionPose() {
        double x = botpose.getPosition().x * 39.3701;
        double y = botpose.getPosition().y * 39.3701;
        double heading_deg = botpose.getOrientation().getYaw();
        double heading_rad = Math.toRadians(heading_deg);
        return new Pose(x, y, heading_rad);
    }

    public boolean shouldUpdate() {
        if (Math.hypot(follower.velocity().vx, follower.velocity().vy) > 2.0) return false;
        if (follower.velocity().omega > 1.0) return false;
        if (current_result.getFiducialResults().size() < 2) return false;

        /*List<LLResultTypes.FiducialResult> results = current_result.getFiducialResults();
        for (int i = 0; i < results.size(); i++) {
            double pitch = results.get(i).getTargetPoseRobotSpace().getOrientation().getPitch();
            if (pitch > ACCEPTABLE_FIDUCIAL_PITCH + ACCEPTABLE_FIDUCIAL_PITCH_PM_RANGE || pitch < ACCEPTABLE_FIDUCIAL_PITCH - ACCEPTABLE_FIDUCIAL_PITCH_PM_RANGE) {
                return false;
            }
        } */

        return true;
    }

    public void updateRobotPoseToVisionPose() { follower.setPose(getVisionPose()); }

    public Follower follower() {
        return follower;
    }

    @Override
    public void periodic() {
        int pipelineIndex = getPipelineByPose();
        if (limeLight.getStatus().getPipelineIndex() != pipelineIndex) limeLight.pipelineSwitch(pipelineIndex);

        double robotYawDeg = Math.toDegrees(follower.pose().heading());
        limeLight.updateRobotOrientation(robotYawDeg);
        current_result = limeLight.getLatestResult();

        botpose = current_result.getBotpose();

        // Maybe move to drivetrain?
        if (ACTIVE && shouldUpdate()) {
            updateRobotPoseToVisionPose();
        }
    }

    @Override
    public void debug() {
        r.telemetry.addLine("== Lime Light ==");

        List<LLResultTypes.FiducialResult> res = current_result.getFiducialResults();
        for (int i = 0; i < res.toArray().length; i++) {
            r.telemetry.addData("Tag id ", res.get(i).getFiducialId());
            r.telemetry.addData("Pitch ", res.get(i).getTargetPoseRobotSpace().getOrientation().getPitch());
        }

        r.telemetry.addData("Botpose", getVisionPose());
        r.telemetry.addData("PedroPose", follower.pose());
    }
}
