package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ZMotor;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.hardware.Falcon;
import frc.libzodiac.hardware.Pro775;

public final class Intake extends SubsystemBase implements ZmartDash {
    public final ZMotor convey = new Falcon(31);
    public final Pro775.Servo lift = new Pro775.Servo(38);
    private final DigitalInput topLimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(1);

    public Intake() {
        this.lift.set_pid(0.5, 5e-5, 50);
        this.lift.profile.put("down", 3850.0);
        this.lift.profile.put("up", 0.0);
        this.lift.profile.put("standby", 1900.0); //todo
    }

    public Intake init() {
        this.convey.init();
        this.lift.init();
        return this;
    }

    public Intake standby() {
        if ((this.lift.get() < this.lift.profile.get("standby") && this.bottomLimitSwitch.get())
                || (this.lift.get() > this.lift.profile.get("standby") && this.topLimitSwitch.get())) {
            this.lift.go("standby");
        }
        this.convey.shutdown();
        this.debug("state", "standby");
        return this;
    }

    public Intake take() {
        if (this.bottomLimitSwitch.get()) {
            if (this.lift.get() < 5000)
                this.lift.go("down");
            else
                this.lift.shutdown();
        }
        this.convey.raw(0.2);
        this.debug("state", "taking");
        return this;
    }

    public Intake send() {
        if (this.lift.get() < 100)
            this.convey.raw(0.2);
        this.debug("pos", this.lift.get());
        this.lift.go("up");
        this.debug("state", "sending");
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
