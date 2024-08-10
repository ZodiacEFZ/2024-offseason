package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.hardware.Falcon;

public final class Shooter extends SubsystemBase implements ZmartDash {

    private final Falcon down = new Falcon(21);
    private final Falcon up = new Falcon(22);

    public Shooter() {
    }

    public Shooter init() {
        this.down.set_pid(0, 0, 0).init();
        this.up.set_pid(0, 0, 0).init();
        return this;
    }

    public Shooter shoot(double speed) {
        this.debug("speed", speed);
        this.down.go(-speed);
        this.up.go(speed);
        return this;
    }

    public Shooter stop() {
        this.down.stop();
        this.up.stop();
        return this;
    }

    @Override
    public String key() {
        return "Shooter";
    }
}

