package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import static com.pedropathing.ivy.commands.Commands.*;

public class Intake {
    private final OpMode o;
    private State state = State.IDLE;

    private final DcMotor intake;
    private final CRServo lServo, rServo;


    public Intake(OpMode o) {
        this.o = o;

        intake = o.hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(Direction.FORWARD);

        lServo = o.hardwareMap.get(CRServo.class, "lIntake");
        lServo.setDirection(Direction.FORWARD);

        rServo = o.hardwareMap.get(CRServo.class, "rIntake");
        rServo.setDirection(Direction.FORWARD);
    }

    private enum State {
        IN(1), OUT(-1), IDLE(0);

        State(int power) {
            this.power = power;
        }

        private final int power;
    }

    public Command update() {
        return infinite(() -> {
            intake.setPower(state.power);
        });
    }

    public Command in() {
        return instant(() -> state = State.IN);
    }

    public Command out() {
        return instant(() -> state = State.OUT);
    }

    public Command idle() {
        return instant(() -> state = State.IDLE);
    }
}
