package frc.robot.commands;

import frc.libzodiac.ZCommand;
import frc.libzodiac.Zoystick;
import frc.libzodiac.util.Vec2D;
import frc.robot.subsystems.Chassis;

public class Example extends ZCommand {

    private final Chassis chassis;

    private Zoystick joystick;

    public Example(Chassis chassis, Zoystick joystick) {
        this.chassis = this.require(chassis);
    }

    @Override
    protected ZCommand exec() {
        if (this.joystick.button("X")) {
            this.chassis.headless();
            this.chassis.go(new Vec2D(1, 0), 0);
        }
        return this;
    }

}
