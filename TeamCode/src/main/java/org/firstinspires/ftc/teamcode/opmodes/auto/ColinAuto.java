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

public class ColinAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        waitForStart();

        robot.start();

        Scheduler.schedule(sequential(
                robot.shootAll(), // Shoot preload

                robot.descorer.deploy(),
                robot.follow(Paths.startToFarFlower()), // Go to far flower
                robot.intake.in(),
                robot.shootAll(Constants.FLOWER_INTAKE_TIME), // Intake and shoot far flower
                robot.intake.idle(),
                robot.follow(Paths.farFlowerToAllianceFLower()), // Go to alliance flower

                race( // Intake from alliance flower (descorer is already down)
                        robot.collectGround(() -> false),
                        waitMs(Constants.FLOWER_INTAKE_TIME)
                ),

                robot.descorer.stow(),

                robot.follow(Paths.allianceFlowerToLeftHive()), // Go to left hive

                robot.shootAll(), // Shoot alliance flower balls into hive

                race(
                        robot.collectGround(() -> false),
                        robot.follow(Paths.leftHiveToGarden())
                ),

                robot.disable()
        ));

        while (opModeIsActive()) {
            robot.periodic();
            robot.debug();
        }
    }

    public static class Paths {
        private static final PoseFactory f = PoseFactory.degrees();

        private static final Pose startToFarFlowerIntermediateStart = f.of(59, 8.73, 180);
        private static final Pose startToFarFlowerIntermediate = f.of(59, 46, 90);
        private static final Pose startIntermediateToFarFlower = f.of(47, 127, 90);
        private static final Pose startIntermediateToFarFlowerControl1 = f.of(59, 115, 0);
        private static final Pose farFlowerToAllianceFlowerIntermediate = f.of(59, 100, -90);
        private static final Pose toAllianceFlower = f.of(16.7331, 47.3075, 177.8336);
        private static final Pose toAllianceFlowerControl1 = f.of(73.6, 7, 0);
        private static final Pose toAllianceFlowerControl2 = f.of(35, 47, 0);
        private static final Pose backToShootInLeftHive = f.of(61.5379, 79.473, -87.0409);
        private static final Pose backToShootInLeftHiveControl1 = f.of(39.0608, 46.8662, 0);
        private static final Pose backToShootInLeftHiveControl2 = f.of(65.1079, 15.1619, 0);
        private static final Pose actuallyBackToShootStart = f.of(61.5379, 79.473, -90);
        private static final Pose actuallyBackToShoot = f.of(60.2021, 110.3691, 180);
        private static final Pose turnTime = f.of(62.3065, 79.4082, -90);
        private static final Pose goToGarden = f.of(8.75, 8.75, 179.8232);
        private static final Pose goToGardenControl1 = f.of(64.5195, 1.8398, 0);
        private static final Pose goToGardenControl2 = f.of(30.0281, 8.75, 0);

        public static Path startToFarFlower() {
            return path(
                    line(startToFarFlowerIntermediateStart, startToFarFlowerIntermediate).linear(startToFarFlowerIntermediateStart, startToFarFlowerIntermediate),
                    curve(startToFarFlowerIntermediate, startIntermediateToFarFlowerControl1, startIntermediateToFarFlower).linear(startToFarFlowerIntermediate, startIntermediateToFarFlower)
            );
        }

        public static Path farFlowerToAllianceFLower() {
            return path(
                    line(startIntermediateToFarFlower, farFlowerToAllianceFlowerIntermediate).linear(startIntermediateToFarFlower, farFlowerToAllianceFlowerIntermediate),
                    curve(farFlowerToAllianceFlowerIntermediate, toAllianceFlowerControl1, toAllianceFlowerControl2, toAllianceFlower).tangent()
            );
        }

        public static Path allianceFlowerToLeftHive() {
            return path(
                    curve(toAllianceFlower, backToShootInLeftHiveControl1, backToShootInLeftHiveControl2, backToShootInLeftHive).reverseTangent(),
                    line(actuallyBackToShootStart, actuallyBackToShoot).linear(actuallyBackToShootStart, actuallyBackToShoot)
            );
        }

        public static Path leftHiveToGarden() {
            return path(
                    line(actuallyBackToShoot, turnTime).linear(actuallyBackToShoot, turnTime),
                    curve(turnTime, goToGardenControl1, goToGardenControl2, goToGarden).tangent()
            );
        }
    }
}