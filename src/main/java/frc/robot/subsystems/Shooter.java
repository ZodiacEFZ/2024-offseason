package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.hardware.Falcon;

public final class Shooter extends SubsystemBase implements ZmartDash {

    private final Falcon down = new Falcon(21);
    private final Falcon up = new Falcon(22);
    private State state = State.Standby;

    public Shooter() {
    }

    public Shooter init() {
        this.down.set_pid(0.01, 0, 0).init();
        this.up.set_pid(0.01, 0, 0).init();
        return this;
    }

    public Shooter standby() {
        // if (this.state == State.Standby)
        // return this;
        this.state = State.Standby;
        this.debug("state", "standby");
        this.up.shutdown();
        this.down.shutdown();
        return this;
    }

    public Shooter shoot() {
        // if (this.state == State.Shooting)
        // return this;
        this.state = State.Shooting;
        this.debug("state", "shooting");
        this.down.set(-0.8);
        this.up.set(0.8);
        //this.down.go(-100);
        //this.up.go(100);
        return this;
    }

    @Override
    public String key() {
        return "Shooter";
    }

    private enum State {
        Standby,
        Shooting,
    }
}
