package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.hardware.Falcon;

public final class Shooter extends SubsystemBase implements ZmartDash {

    private final Falcon left = new Falcon(30), right = new Falcon(31);

    public Shooter() {
    }

    public Shooter init() {
        this.left.init();
        this.right.init();
        return this;
    }

    public Shooter shoot(double speed) {
        this.debug("speed", speed);
        this.left.go(speed);
        this.right.go(speed);
        return this;
    }

    public Shooter stop() {
        this.left.stop();
        this.right.stop();
        return this;
    }

    @Override
    public String key() {
        return "Shooter";
    }
}

