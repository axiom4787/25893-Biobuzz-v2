package org.firstinspires.ftc.teamcode;


import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerDescorer;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.SOTM;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Constants;

import java.util.List;
import java.util.function.BooleanSupplier;


public class Robot {
    public final OpMode o;
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final Gamepad gamepad;

    private final List<LynxModule> allHubs;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTime = 0;

    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Transfer transfer;
    public final Blocker blocker;
    public final FlowerDescorer descorer;
    public final Shooter shooter;
    public final Turret turret;
    public final Hood hood;
    public final SOTM sotm;
    public final Lights lights;

    public Robot(OpMode o) {
        this.o = o;
        telemetry = new MultipleTelemetry(o.telemetry, FtcDashboard.getInstance().getTelemetry());
        hardwareMap = o.hardwareMap;
        gamepad = o.gamepad1;

        drivetrain = new Drivetrain(this);
        intake = new Intake(this);
        transfer = new Transfer(this);
        blocker = new Blocker(this);
        descorer = new FlowerDescorer(this);
        shooter = new Shooter(this);
        turret = new Turret(this);
        hood = new Hood(this);
        sotm = new SOTM(this);
        lights = new Lights(this);

        allHubs = o.hardwareMap.getAll(LynxModule.class);
        allHubs.forEach((m) -> m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    }

    public void start() {
        Scheduler.reset();

        Scheduler.schedule(infinite(() -> {
            drivetrain.periodic();
            intake.periodic();
            transfer.periodic();
            blocker.periodic();
            descorer.periodic();
            shooter.periodic();
            turret.periodic();
            hood.periodic();
            sotm.periodic();
            lights.periodic();
        }));

        loopTimer.reset();
    }

    public void periodic() {
        allHubs.forEach(LynxModule::clearBulkCache);
        lastLoopTime = loopTimer.milliseconds();
        loopTimer.reset();

        Scheduler.execute();
    }

    public void debug() {
        drivetrain.debug();
        intake.debug();
        transfer.debug();
        blocker.debug();
        descorer.debug();
        shooter.debug();
        turret.debug();
        hood.debug();
        sotm.debug();
        lights.debug();

        telemetry.addData("Misc - Last loop time (ms)", lastLoopTime);
        telemetry.update();
    }

    /// Shoot all possessed game elements (for Constants.SHOOT_TIME)
    public Command shootAll() {
        return shootAll(0);
    }

    /// Shoot all possessed game elements (for Constants.SHOOT_TIME plus additionalShootTime)
    public Command shootAll(double additionalShootTime) {
        return sequential(
//                shooter.enable(),
//                race(
//                        waitUntil(shooter::isReady), // wait to get up to speed
//                        waitMs(800) // wait a max of 800ms to spin up
//                ).unless(shooter::isReady), // ... unless the shooter is already ready
                blocker.unblock(),
                intake.in(),
                transfer.in(),
                waitMs(Constants.SHOOT_TIME + additionalShootTime),
                blocker.block(),
                intake.idle(),
                transfer.idle()
        ).requiring(shooter, transfer, blocker, shooter);
    }

    public Command shoot() {
        return new CommandBuilder()
                .setStart(() -> Scheduler.schedule(
                        blocker.unblock(),
                        intake.in(),
                        transfer.in()
                ))
                .setDone(() -> false)
                .setEnd((endCondition) -> Scheduler.schedule(
                        blocker.block(),
                        intake.idle(),
                        transfer.idle()
                ))
                .requiring(blocker, intake, transfer)
                .setConflictBehavior(ConflictBehavior.CANCEL)
                .setPriority(10);
    }

    public Command uncollect() {
        return new CommandBuilder()
                .setStart(() -> Scheduler.schedule(
                        intake.out(),
                        transfer.out()
                ))
                .setEnd((endCondition) -> Scheduler.schedule(
                        intake.idle(),
                        transfer.idle()
                ))
                .requiring(intake, transfer)
                .setConflictBehavior(ConflictBehavior.CANCEL)
                .setPriority(7);
    }

    public Command collectGround() {
        return new CommandBuilder()
                .setStart(() -> Scheduler.schedule(
                        blocker.block(),
                        intake.in(),
                        transfer.in()
                ))
                .setEnd((endCondition) -> Scheduler.schedule(
                        intake.idle(),
                        transfer.idle()
                ))
                .requiring(intake, transfer, blocker)
                .setConflictBehavior(ConflictBehavior.CANCEL)
                .setPriority(8);
    }

    public Command collectFlower() {
        return new CommandBuilder()
                .setStart(() -> Scheduler.schedule(
                        blocker.block(),
                        intake.in(),
                        transfer.in(),
                        descorer.deploy()
                ))
                .setEnd((endCondition) -> Scheduler.schedule(
                        intake.idle(),
                        transfer.idle(),
                        descorer.stow()
                ))
                .requiring(intake, transfer, blocker, descorer)
                .setConflictBehavior(ConflictBehavior.CANCEL)
                .setPriority(9);
    }

    /// Disable the robot
    public Command disable() {
        return instant(() -> Scheduler.schedule(
                intake.idle(),
                transfer.idle(),
                turret.disable(),
                shooter.disable(),
                instant(() -> drivetrain.follower().stop())
        )).setPriority(25893).requiring(intake, transfer, blocker, descorer, turret, shooter, drivetrain);
    }

    /// Follow a path
    public Command follow(Path path) {
        return PedroCommands.follow(drivetrain.follower(), path);
    }
}
