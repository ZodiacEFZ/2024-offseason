package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.Util;
import frc.libzodiac.ZMotor;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.hardware.Falcon;
import frc.libzodiac.hardware.Pro775;

public final class Intake extends SubsystemBase implements ZmartDash {
    public final ZMotor convey = new Falcon(31);
    public final Pro775.Servo lift = new Pro775.Servo(32);
    private final DigitalInput topLimitSwitch = new DigitalInput(8);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(0);

    public Intake() {
        this.lift.set_pid(0.5, 1e-4, 50);
        this.lift.profile.put("down", 3500.0);
        this.lift.profile.put("up", 0.0);
        this.lift.profile.put("standby", 0.0);
        this.lift.profile.put("amp", 1200.0);
        this.convey.set_pid(0, 0, 0);
    }

    public Intake init() {
        this.convey.init();
        this.lift.init();
        this.lift.reset();
        return this;
    }

    public Intake standby() {
        if ((this.lift.get() < this.lift.profile.get("standby") && this.bottomLimitSwitch.get()) || (this.lift.get() > this.lift.profile.get("standby") && this.topLimitSwitch.get())) {
            this.lift.go("standby");
        }
        this.convey.shutdown();
        return this;
    }

    public Intake up() {
        if (this.topLimitSwitch.get()) {
            this.lift.go("up");
        }
        this.convey.shutdown();
        return this;
    }

    public Intake amp() {
        if ((this.lift.get() < this.lift.profile.get("standby") && this.bottomLimitSwitch.get()) || (this.lift.get() > this.lift.profile.get("standby") && this.topLimitSwitch.get())) {
            this.lift.go("amp");
        }
        if (Util.approx(this.lift.get(), this.lift.profile.get("amp"), 150)) {
            this.convey.raw(0.4);
        }
        return this;
    }

    public Intake take() {
        this.lift.go("down");
        this.convey.raw(-0.4);
        return this;
    }

    public Intake send() {
        if (this.topLimitSwitch.get()) {
            this.lift.go("up");
        }
        if (!this.topLimitSwitch.get() || this.lift.get() < 100) {
            this.convey.raw(0.3);
        }
        this.debug("pos", this.lift.get());
        return this;
    }

    @Override
    public void periodic() {
    }

    @Override
    public String key() {
        return "Intake";
    }
}
