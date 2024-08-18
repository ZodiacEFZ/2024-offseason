package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.hardware.Falcon;

public final class Shooter extends SubsystemBase implements ZmartDash {

    private final Falcon rd = new Falcon(21);
    private final Falcon ru = new Falcon(22);
    private final Falcon ld = new Falcon(23);
    private final Falcon lu = new Falcon(24);

    public Shooter init() {
        this.rd.set_pid(0.01, 0, 0).init().stop(false);
        this.ru.set_pid(0.01, 0, 0).init().stop(false);
        this.ld.set_pid(0.01, 0, 0).init().stop(false);
        this.lu.set_pid(0.01, 0, 0).init().stop(false);
        return this;
    }

    public Shooter standby() {
        this.debug("state", "standby");
        this.rd.shutdown();
        this.ru.shutdown();
        this.ld.shutdown();
        this.lu.shutdown();
        return this;
    }

    public Shooter shoot() {
        this.debug("state", "shooting");
        this.ld.go(5000);
        this.lu.go(5000);
        this.rd.go(5000);
        this.ru.go(5000);
        return this;
    }

    @Override
    public String key() {
        return "Shooter";
    }
}
