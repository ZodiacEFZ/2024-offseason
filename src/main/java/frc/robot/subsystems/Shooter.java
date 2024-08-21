package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ZCommand;
import frc.libzodiac.Zambda;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.hardware.Falcon;
import frc.libzodiac.ui.Axis;

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
        this.rd.shutdown();
        this.ru.shutdown();
        this.ld.shutdown();
        this.lu.shutdown();
        return this;
    }

    public Shooter shoot() {
        return this.shoot(-5000);
    }

    public Shooter shoot(double speed) {
        this.ld.go(-speed);
        this.lu.go(speed);
        this.rd.go(speed);
        this.ru.go(-speed);
        return this;
    }

    public ZCommand ctrl(Axis output) {
        final var speed = output.map(x -> x < 0 ? x : 0).threshold(.1);
        return new Zambda(this, () -> {
            final var v = speed.get();
            if (v == 0) {
                this.standby();
            } else {
                this.shoot(v * 5000);
            }
        });
    }

    @Override
    public String key() {
        return "Shooter";
    }
}
