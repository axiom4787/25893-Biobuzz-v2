package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.FlowerDescorer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

public class Robot {
    private final OpMode o;

    private final List<LynxModule> allHubs;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTime = 0;

    public final Intake intake;
    public final Transfer transfer;
    public final Blocker blocker;
    public final FlowerDescorer descorer;
    public final Shooter shooter;
    public final Turret turret;

    public Robot(OpMode o) {
        this.o = o;

        intake = new Intake(o);
        transfer = new Transfer(o);
        blocker = new Blocker(o);
        descorer = new FlowerDescorer(o);
        shooter = new Shooter(o);
        turret = new Turret(o);

        allHubs = o.hardwareMap.getAll(LynxModule.class);
        allHubs.forEach((m) -> m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    }

    public void start() {
        Scheduler.schedule(
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
