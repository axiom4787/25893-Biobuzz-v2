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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Drivetrain extends Subsystem {
    private final Follower follower;

    public Drivetrain(OpMode o) {
        super(o);

        follower = Constants.create(o.hardwareMap);
    }

    public void drive(DrivePowers powers, boolean hold) {
        if (hold) {
            ManualDrive.driveOrHold(follower, powers);
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

    public Follower follower() { return follower; }

    @Override
    public Command periodicCommand() {
        return infinite(follower::update);
    }

    @Override
    public void debug() {
        o.telemetry.addLine("Drivetrain:");
        o.telemetry.addData(
                "- Position",
                "X=%3.2f, Y=%3.2f, H=%5.2f°",
                getPose().x(),
                getPose().y(),
                getPose().heading()
        );
        o.telemetry.addData(
                "- Velocity",
                "X=%2.2f, Y=%2.2f, H=%3.2f",
                follower.velocity().vx,
                follower.velocity().vy,
                follower.velocity().omega
        );
        o.telemetry.addData("- Follower state", follower.mode().name());
    }
}
