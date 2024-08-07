package frc.robot.commands;

import frc.libzodiac.ZCommand;
import frc.libzodiac.Zoystick;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public final class Shoot extends ZCommand {

    private final Intake intake;
    private final Shooter shooter;
    private final Zoystick ctrl;

    public Shoot(Intake intake, Shooter shooter, Zoystick ctrl) {
        this.intake = this.require(intake);
        this.shooter = this.require(shooter);
        this.ctrl = ctrl;
    }

    @Override
    protected ZCommand exec() {
        final var v = this.ctrl.rTrigger();
        if (v < 0.001) {
            this.shooter.stop();
            this.intake.lift.go("standby");
            return this;
        }
        this.shooter.shoot(v);
        this.intake.lift.go("up");
        this.intake.convey.go("out");
        return this;
    }
}
