package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.skeletonarmy.marrow.zones.Point;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Subsystem;

import java.util.List;


public class LimeLight extends Subsystem {
    private static final int PIPELINE_INDEX = 0;
    private static final double MINIMUM_CONF = 0.7;

    private final Follower follower;
    private final Limelight3A limeLight;

    private LLResult current_result;
    private Pose3D botpose;

    public LimeLight(Robot r, Follower follower) {
        super(r);

        this.follower = follower;
        this.limeLight = r.hardwareMap.get(Limelight3A.class, "limelight");

        LLStatus status = limeLight.getStatus();
        if (status.getPipelineIndex() != PIPELINE_INDEX) { limeLight.pipelineSwitch(PIPELINE_INDEX); }

        limeLight.start();
    }

    public Pose getVisionPose() {
        double x = botpose.getPosition().x;
        double y = botpose.getPosition().y;
        double heading_deg = botpose.getOrientation().getYaw();
        double heading_rad = Math.toRadians(heading_deg);
        return new Pose(x, y, heading_rad);
    }

    public boolean shouldUpdate() {
        if (Math.hypot(follower.velocity().vx, follower.velocity().vy) > 2.0) return false;
        if (follower.velocity().omega > 1.0) return false;
        if (current_result.getFiducialResults().size() < 2) return false;

        return true;
    }

    public void updateRobotPoseToVisionPose() { follower.setPose(getVisionPose()); }

    public Follower follower() {
        return follower;
    }

    @Override
    public void periodic() {
        double robotYawDeg = Math.toDegrees(follower.pose().heading());
        limeLight.updateRobotOrientation(robotYawDeg);
        current_result = limeLight.getLatestResult();

        botpose = current_result.getBotpose_MT2();
    }

    @Override
    public void debug() {
        r.telemetry.addLine("== Lime Light ==");
        r.telemetry.addData("PythonOutput", java.util.Arrays.toString(current_result.getPythonOutput()));

        r.telemetry.addData("tx", current_result.getTx());
        r.telemetry.addData("txnc", current_result.getTxNC());
        r.telemetry.addData("ty", current_result.getTy());
        r.telemetry.addData("tync", current_result.getTyNC());

        List<LLResultTypes.FiducialResult> res = current_result.getFiducialResults();
        for (int i = 0; i < res.toArray().length; i++) {
            r.telemetry.addData("Tag id ", res.get(i).getFiducialId());
            r.telemetry.addData("Pitch ", res.get(i).getTargetPoseRobotSpace().getOrientation().getPitch());
        }

        r.telemetry.addData("Botpose", botpose.toString());
    }
}
