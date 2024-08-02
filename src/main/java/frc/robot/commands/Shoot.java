package frc.robot.commands;

import frc.libzodiac.ZCommand;
import frc.libzodiac.Zoystick;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class Shoot extends ZCommand {

    private final Intake intake;
    private final Shooter shooter;
    private final Zoystick drive;

    public Shoot(Intake intake, Shooter shooter, Zoystick drive) {
        this.intake = this.require(intake);
        this.shooter = this.require(shooter);
        this.drive = drive;
    }

    @Override
    protected ZCommand exec() {
        var speed = drive.rTrigger();
        //shooter.shoot(speed);
        if(speed>0)
        {
            intake.output(speed);
        }
        return this;
    }
}
