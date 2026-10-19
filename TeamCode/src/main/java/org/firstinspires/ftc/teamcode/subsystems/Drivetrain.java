package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Drivetrain extends Subsystem {
    private final Follower follower;

    public Drivetrain(Robot r) {
        super(r);

        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(r.hardwareMap);
    }

    public void drive(DrivePowers powers, boolean hold) {
        if (hold) {
            ManualDrive.driveOrHold(follower, powers, 0.0, 0.2);
        } else {
            follower.manual(powers);
        }
    }

    public Pose getPose() { return follower.pose(); }

    public void setPose(Pose pose) { follower.setPose(pose); }

    public void follow(Path path) {
        Scheduler.schedule(
                PedroCommands.follow(follower, path)
        );
    }

    public double getDistanceToGoal() {
        Pose robotPose = getPose();
        Pose goalPose = robotPose.y() > Constants.HALF_FIELD_WIDTH ? Constants.LEFT_GOAL : Constants.RIGHT_GOAL;

        double dx = robotPose.x() - goalPose.x();
        double dy = robotPose.y() - goalPose.y();

        return Math.hypot(dx, dy);
    }

    public boolean isInScoringZone() {
        PolygonZone robotZone = new PolygonZone(16, 16);
        robotZone.setRotation(getPose().heading());
        return robotZone.isFullyInside(Constants.LEFT_SCORING_ZONE) || robotZone.isFullyInside(Constants.LEFT_SCORING_ZONE);
    }

    public Follower follower() { return follower; }

    @Override
    public Command periodicCommand() {
        return infinite(follower::update);
    }

    @Override
    public void debug() {
        r.telemetry.addLine("Drivetrain:");
        r.telemetry.addData(
                "- Position",
                "X=%3.2f, Y=%3.2f, H=%5.2f°",
                getPose().x(),
                getPose().y(),
                getPose().heading()
        );
        r.telemetry.addData(
                "- Velocity",
                "X=%2.2f, Y=%2.2f, H=%3.2f",
                follower.velocity().vx,
                follower.velocity().vy,
                follower.velocity().omega
        );
        r.telemetry.addData("- Follower state", follower.mode().name());
    }
}
