package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Scheduler;
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

import java.util.List;

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

        Scheduler.schedule(
                drivetrain.periodicCommand(),
                intake.periodicCommand(),
                transfer.periodicCommand(),
                blocker.periodicCommand(),
                descorer.periodicCommand(),
                shooter.periodicCommand(),
                turret.periodicCommand()
        );

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

        o.telemetry.addData("Misc - Last loop time (ms)", lastLoopTime);
    }
}
