package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.libzodiac.ZCommand;
import frc.libzodiac.util.Vec2D;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.LinkedList;
import java.util.Queue;

public class Auto extends ZCommand {
    private static final double CHASSIS_POSITION_KP = 0.1;
    private static final double CHASSIS_ROTATION_KP = 1;
    private static final double CHASSIS_POSITION_THRESHOLD = 1;
    private static final double CHASSIS_ROTATION_THRESHOLD = 0.05;
    private static final Timer timer = new Timer();
    private static Chassis chassis;
    private static Intake intake;
    private static Shooter shooter;
    public static final AutoCommand Left = new AutoCommand() {
        @Override
        public AutoCommand init() {
            commands.add(() -> {
                if (timer.get() < 1.5) {
                    intake.send();
                    shooter.shoot();
                    return false;
                }
                intake.standby();
                shooter.standby();
                return true;
            });
            commands.add(() -> go(new Vec2D(3, 3), 0));
            commands.add(() -> {
                intake.take();
                return go(new Vec2D(4, 4), 0);
            });
            commands.add(() -> {
                if (timer.get() < 0.5) {
                    intake.take();
                    return false;
                }
                intake.standby();
                return true;
            });
            commands.add(() -> go(new Vec2D(1, 1), 0));
            commands.add(() -> {
                if (timer.get() < 1.5) {
                    intake.send();
                    shooter.shoot();
                    return false;
                }
                intake.standby();
                shooter.standby();
                return true;
            });
            commands.add(() -> go(new Vec2D(3, 3), 0));
            return this;
        }
    };
    private static AutoCommand command;

    public Auto(Chassis chassis, Intake intake, Shooter shooter, AutoCommand command) {
        Auto.chassis = require(chassis);
        Auto.intake = require(intake);
        Auto.shooter = require(shooter);
        Auto.command = command;
    }

    private static boolean go(Vec2D pos, double yaw) {
        final var deltaPos = pos.sub(Chassis.inav.getPosition());
        final var deltaYaw = yaw - Chassis.inav.getYaw();
        chassis.go(deltaPos.mul(CHASSIS_POSITION_KP), deltaYaw * CHASSIS_ROTATION_KP);
        return deltaPos.r() < CHASSIS_POSITION_THRESHOLD && Math.abs(deltaYaw) < CHASSIS_ROTATION_THRESHOLD;
    }

    public Auto init() {
        command.init();
        timer.start();
        return this;
    }

    @Override
    protected ZCommand exec() {
        command.run();
        return this;
    }

    public interface AutoCommand {
        Queue<AutoLambda> commands = new LinkedList<>();

        AutoCommand init();

        default AutoCommand run() {
            if (!commands.isEmpty()) {
                if (commands.peek().run()) {
                    commands.poll();
                    timer.reset();
                }
            }
            return this;
        }
    }

    public interface AutoLambda {
        boolean run();
    }
}
