package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Auto1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        waitForStart();

        robot.start();

        Scheduler.schedule(sequential(
                robot.shootAll(),

                race( // Go to the garden (and wait a little) while collecting pollen from the ground
                        sequential(
                                robot.follow(Paths.START_TO_GARDEN),
                                waitMs(Constants.GARDEN_INTAKE_TIME)
                        ),
                        robot.collectGround(() -> false)
                ),

                robot.follow(Paths.GARDEN_TO_LEFT_HIVE_SCORE),
                robot.shootAll(),

                race( // Move to alliance wall flower and descore it by running the intake and transfer
                        sequential(
                                robot.follow(Paths.LEFT_HIVE_SCORE_TO_ALLIANCE_FLOWER),
                                waitMs(Constants.FLOWER_INTAKE_TIME)
                        ),
                        robot.collectFlower(() -> false)
                ),

                robot.follow(Paths.ALLIANCE_FLOWER_TO_LEFT_HIVE_SCORE),
                robot.shootAll(),

                robot.descorer.deploy(),
                robot.follow(Paths.LEFT_HIVE_SCORE_TO_OTHER_FLOWER),
                robot.intake.in(),
                robot.shootAll(Constants.FLOWER_INTAKE_TIME),
                robot.intake.idle(),
                waitMs(Constants.FLOWER_INTAKE_TIME + Constants.SHOOT_TIME),
                robot.follow(Paths.OTHER_FLOWER_TO_PARK),
                robot.descorer.stow(),

                robot.disable()
        ));

        while (opModeIsActive()) {
            robot.periodic();
            robot.debug();
        }
    }

    private static class Paths {
        private static final PoseFactory f = PoseFactory.degrees();

        private static final Pose START = f.of(58, 8.75, 180);
        private static final Pose START_GARDEN_CTRL = f.of(15, 32, 0);
        private static final Pose GARDEN = f.of(8.75, 8.75, -90);
        private static final Path START_TO_GARDEN = curve(START, START_GARDEN_CTRL, GARDEN).linear(START, GARDEN);
        private static final Pose LEFT_HIVE_ALLIANCE_FLOWER_CTRL = f.of(32, 47, 0);
        private static final Pose ALLIANCE_FLOWER = f.of(15, 47, 180);
        private static final Pose LEFT_HIVE_SCORE = f.of(39, 111, 90);

        private static final Path GARDEN_TO_LEFT_HIVE_SCORE = line(GARDEN, LEFT_HIVE_SCORE).linear(GARDEN, LEFT_HIVE_SCORE);
        private static final Path LEFT_HIVE_SCORE_TO_ALLIANCE_FLOWER = curve(LEFT_HIVE_SCORE, LEFT_HIVE_ALLIANCE_FLOWER_CTRL, ALLIANCE_FLOWER).linear(LEFT_HIVE_SCORE, ALLIANCE_FLOWER);
        private static final Path ALLIANCE_FLOWER_TO_LEFT_HIVE_SCORE = line(ALLIANCE_FLOWER, LEFT_HIVE_SCORE).linear(ALLIANCE_FLOWER, LEFT_HIVE_SCORE.withHeading(90));
        private static final Pose OTHER_FLOWER = f.of(46, 126, 90);
        private static final Path LEFT_HIVE_SCORE_TO_OTHER_FLOWER = line(LEFT_HIVE_SCORE, OTHER_FLOWER).linear(LEFT_HIVE_SCORE.withHeading(90), OTHER_FLOWER);
        private static final Pose PARK = f.of(13, 97, 90);
        private static final Path OTHER_FLOWER_TO_PARK = line(OTHER_FLOWER, PARK).linear(OTHER_FLOWER, PARK);
    }
}
