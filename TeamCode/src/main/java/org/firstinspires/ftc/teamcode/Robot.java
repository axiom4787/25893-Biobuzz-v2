package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerDescorer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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

    public Robot(OpMode o) {
        this.o = o;
        telemetry = o.telemetry;
        hardwareMap = o.hardwareMap;

        drivetrain = new Drivetrain(this);
        intake = new Intake(this);
        transfer = new Transfer(this);
        blocker = new Blocker(this);
        descorer = new FlowerDescorer(this);
        shooter = new Shooter(this);
        turret = new Turret(this);

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
        intake.debug();
        transfer.debug();
        blocker.debug();
        descorer.debug();
        shooter.debug();
        turret.debug();

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
                shooter.enable(),
                race(
                        waitUntil(shooter::isReady), // wait to get up to speed
                        waitMs(800) // wait a max of 800ms to spin up
                ).unless(shooter::isReady), // ... unless the shooter is already ready
                transfer.in(),
                blocker.unblock(),
                waitMs(Constants.SHOOT_TIME + additionalShootTime),
                transfer.idle(),
                blocker.block()
        ).requiring(shooter, transfer);
    }

    /// @param isDone Whether to stop attempting to collect pollen from the ground (ex: () -> gamepad1.aWasReleased())
    public Command collectGround(BooleanSupplier isDone) {
        return new CommandBuilder()
                .setStart(() -> Scheduler.schedule(
                        intake.in(),
                        transfer.in()
                ))
                .setEnd((endCondition) -> Scheduler.schedule(
                        intake.idle(),
                        transfer.idle()
                ))
                .setDone(isDone)
                .requiring(intake, transfer);
    }

    /// @param isDone Whether to stop attempting to collect pollen from a flower (ex: () -> gamepad1.aWasReleased())
    public Command collectFlower(BooleanSupplier isDone) {
        return new CommandBuilder()
                .setStart(() -> Scheduler.schedule(
                        intake.in(),
                        transfer.in(),
                        descorer.deploy()
                ))
                .setEnd((endCondition) -> Scheduler.schedule(
                        intake.idle(),
                        transfer.idle(),
                        descorer.stow()
                ))
                .setDone(isDone)
                .requiring(intake, transfer, descorer);
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
