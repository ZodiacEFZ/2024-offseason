package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.libzodiac.ZCommand;
import frc.libzodiac.util.Vec2D;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Auto extends ZCommand {
    private static final double CHASSIS_POSITION_KP = 0.1;
    private static final double CHASSIS_ROTATION_KP = 1;

    private static final double CHASSIS_POSITION_THRESHOLD = 1;
    private static final double CHASSIS_ROTATION_THRESHOLD = 0.05;

    public Chassis chassis;
    public Intake intake;
    public Shooter shooter;
    public Position position;

    private final Timer timer = new Timer();

    private boolean position_desired = false;
    private State state;

    public Auto(Chassis chassis, Intake intake, Shooter shooter, Position position) {
        this.chassis = require(chassis);
        this.intake = require(intake);
        this.shooter = require(shooter);
        this.position = position;
    }

    public Auto init() {
        this.state = State.Shoot;
        this.timer.start();
        return this;
    }

    private Auto next() {
        this.timer.reset();
        this.state = switch (this.state) {
            case Shoot -> State.Move;
            case Move -> State.Intake;
            case Intake -> State.MoveBack;
            case MoveBack -> State.ShootAgain;
            case ShootAgain -> State.Leave;
            case Leave, Done -> State.Done;
        };
        this.position_desired = false;
        return this;
    }

    private boolean go(Vec2D pos, double yaw) {
        final var deltaPos = pos.sub(Chassis.inav.getPosition());
        final var deltaYaw = yaw - Chassis.inav.getYaw();
        if (deltaPos.r() < CHASSIS_POSITION_THRESHOLD && Math.abs(deltaYaw) < CHASSIS_ROTATION_THRESHOLD) {
            return true;
        }
        this.chassis.go(deltaPos.mul(CHASSIS_POSITION_KP), deltaYaw * CHASSIS_ROTATION_KP);
        return false;
    }

    private Auto shoot() {
        if (this.timer.get() < 1.5) {
            this.intake.send();
            this.shooter.shoot();
        } else {
            this.intake.standby();
            this.shooter.standby();
            this.next();
        }
        return this;
    }

    private Auto go_until(Vec2D pos, double yaw) {
        return this.go(pos, yaw) ? this.next() : this;
    }

    private ZCommand left() {
        // TODO: measure real position
        final var p_move = new Vec2D(3, 3);
        final var p_move_back = new Vec2D(0, 0);
        final var p_leave = new Vec2D(5, 5);

        return switch (this.state) {
            case Shoot, ShootAgain -> this.shoot();
            case Move -> this.go_until(p_move, 0);
            case Intake -> {
                final var targetPos = new Vec2D(4, 4);
                final var targetYaw = 0;
                intake.take();
                if (go(targetPos, targetYaw) && !position_desired) {
                    timer.reset();
                    position_desired = true;
                }
                if (timer.get() > 0.5) {
                    intake.standby();
                    next();
                }
                yield this;
            }
            case MoveBack -> this.go_until(p_move_back, 0);
            case Leave -> this.go_until(p_leave, 0);
            case Done -> this;
        };
    }

    public ZCommand center() {
        return this;
    }

    private ZCommand right() {
        return this;
    }

    @Override
    protected ZCommand exec() {
        return switch (position) {
            case Left -> left();
            case Center -> center();
            case Right -> right();
        };
    }

    public enum Position {
        Left,
        Center,
        Right
    }

    private enum State {
        Shoot,
        Move,
        Intake,
        MoveBack,
        ShootAgain,
        Leave,
        Done
    }
}
