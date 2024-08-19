package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.libzodiac.ZCommand;
import frc.libzodiac.util.Vec2D;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Auto extends ZCommand {
    private static final double CHASSIS_POSITION_KP = 0.1;
    private static final double CHASSIS_ROTATION_KP = 1;
    private static final Timer timer = new Timer();
    private static final double CHASSIS_POSITION_THREHOLD = 1;
    private static final double CHASSIS_ROTATION_THRESHOLD = 0.05;
    public static Chassis chassis;
    public static Intake intake;
    public static Shooter shooter;
    public static POSITION position;
    private static boolean auto_tag = false;
    private static Auto instance;
    private static STATE state;

    private Auto(Chassis chassis, Intake intake, Shooter shooter, POSITION position) {
        Auto.chassis = require(chassis);
        Auto.intake = require(intake);
        Auto.shooter = require(shooter);
        Auto.position = position;
    }

    public static Auto getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Auto not initialized");
        }
        return instance;
    }

    public static Auto initInstance(Chassis chassis, Intake intake, Shooter shooter, POSITION position) {
        instance = new Auto(chassis, intake, shooter, position);
        state = STATE.SHOOT;
        timer.start();
        return instance;
    }

    private void next() {
        timer.reset();
        state = switch (state) {
            case SHOOT -> STATE.MOVE;
            case MOVE -> STATE.INTAKE;
            case INTAKE -> STATE.MOVE_BACK;
            case MOVE_BACK -> STATE.SHOOT_AGAIN;
            case SHOOT_AGAIN -> STATE.LEAVE;
            case LEAVE, DONE -> STATE.DONE;
        };
        auto_tag = false;
    }

    private boolean go(Vec2D pos, double yaw) {
        final var deltaPos = pos.sub(Chassis.inav.getPosition());
        final var deltaYaw = yaw - Chassis.inav.getYaw();
        if (deltaPos.r() < CHASSIS_POSITION_THREHOLD && Math.abs(deltaYaw) < CHASSIS_ROTATION_THRESHOLD) {
            return true;
        }
        chassis.go(deltaPos.mul(CHASSIS_POSITION_KP), deltaYaw * CHASSIS_ROTATION_KP);
        return false;
    }

    private ZCommand left() {
        switch (state) {
            case SHOOT -> {
                intake.send();
                shooter.shoot();
                if (timer.get() > 1.5) {
                    intake.standby();
                    shooter.standby();
                    next();
                }
            }
            case MOVE -> {
                final var targetPos = new Vec2D(3, 3);
                final var targetYaw = 0;
                if (go(targetPos, targetYaw)) {
                    next();
                }
            }
            case INTAKE -> {
                final var targetPos = new Vec2D(4, 4);
                final var targetYaw = 0;
                intake.take();
                if (go(targetPos, targetYaw) && !auto_tag) {
                    timer.reset();
                    auto_tag = true;
                }
                if (timer.get() > 0.5) {
                    intake.standby();
                    next();
                }
            }
            case MOVE_BACK -> {
                final var targetPos = new Vec2D(0, 0);
                final var targetYaw = 0;
                if (go(targetPos, targetYaw)) {
                    next();
                }
            }
            case SHOOT_AGAIN -> {
                intake.send();
                shooter.shoot();
                if (timer.get() > 1.5) {
                    intake.standby();
                    shooter.standby();
                    next();
                }
            }
            case LEAVE -> {
                final var targetPos = new Vec2D(3, 3);
                final var targetYaw = 0;
                if (go(targetPos, targetYaw)) {
                    next();
                }
            }
            case DONE -> {
            }
        }
        return instance;
    }

    public ZCommand center() {
        return instance;
    }

    private ZCommand right() {
        return instance;
    }

    @Override
    protected ZCommand exec() {
        return switch (position) {
            case LEFT -> left();
            case CENTER -> center();
            case RIGHT -> right();
        };
    }

    public enum POSITION {
        LEFT,
        CENTER,
        RIGHT
    }

    private enum STATE {
        SHOOT,
        MOVE,
        INTAKE,
        MOVE_BACK,
        SHOOT_AGAIN,
        LEAVE,
        DONE
    }
}
