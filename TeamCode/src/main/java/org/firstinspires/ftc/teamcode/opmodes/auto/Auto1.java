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

                robot.follow(Paths.GeorgeLScore),

                sequential(
                        waitMs(1000),
                        robot.follow(Paths.LScoreFred),
                        robot.follow(Paths.FredPark)
                ).raceWith(
                        robot.collectFlower()
                ),

                robot.shootAll(),

                robot.disable()
        ));

        while (opModeIsActive()) {
            robot.periodic();
            robot.debug();
        }
    }

    /* ============================================================= *
     *                 Turtle Tracer — Auto-Generated                *
     *                                                               *
     *  Version: 2.6.1.                                              *
     *  Copyright (c) 2026 Matthew Allen                             *
     *                                                               *
     *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
     *  Changes will be overwritten when regenerated.                *
     * ============================================================= */

    private static final PoseFactory p = PoseFactory.degrees();

    public static class Paths {
        public static Path StartGeorge = line(
                p.of(25.000, 9.250, 180),
                p.of(11.500, 9.250, 0.0)
        ).constant(Math.toRadians(180));
        public static Path GeorgeLScore = curve(
                p.of(10.250, 9.250, 0.0),
                p.of(23.708, 72.579, 0.0),
                p.of(19.163, 55.763, 0.0),
                p.of(48.500, 123.000, 0.0)
        ).linear(Math.toRadians(180), Math.toRadians(90));
        public static Path LScoreFred = line(
                p.of(48.500, 123.000, 0.0),
                p.of(48.500, 128.250, 0.0)
        ).linear(Math.toRadians(90), Math.toRadians(90));
        public static Path FredPark = curve(
                p.of(48.500, 128.250, 0.0),
                p.of(38.422, 87.991, 0.0),
                p.of(26.546, 88.481, 0.0),
                p.of(12.500, 90.500, 0.0)
        ).linear(Math.toRadians(90), Math.toRadians(0));
    }
}
