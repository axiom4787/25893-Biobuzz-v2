package org.firstinspires.ftc.teamcode.opmodes.auto;


import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;


@Autonomous
public class Auto1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        waitForStart();

        robot.start();
        robot.drivetrain.setPose(p.of(25.000, 9.250, 180));

        Scheduler.schedule(sequential(
                robot.turret.enable(),

                robot.shooter.enable(),
                waitMs(2000),
                robot.shootAll(),

                sequential(
                        robot.follow(Paths.StartGeorge),
                        waitMs(Constants.GARDEN_INTAKE_TIME)
                ).raceWith(
                        robot.collectGround()
                ),

                robot.shootAll(),

                robot.follow(Paths.GeorgePark),

                robot.turret.zero_(),

                waitMs(2000),

                robot.disable()
        ));

        while (opModeIsActive()) {
            robot.periodic();
            robot.debug();
        }
    }

    private static final PoseFactory p = PoseFactory.degrees();

    public static class Paths {

        public static Path StartGeorge = line(
                p.of(25.000, 9.250, 180.0),
                p.of(11.500, 9.250, 0.0)
        ).constant(Math.toRadians(180));

        public static Path GeorgePark = curve(
                p.of(11.500, 9.250, 0.0),
                p.of(31.947, 49.422, 0.0),
                p.of(14, 90.500, 0.0)
        ).constant(Math.toRadians(180));
    }

}
