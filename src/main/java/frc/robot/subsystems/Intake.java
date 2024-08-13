package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ZMotor;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.hardware.Falcon;
import frc.libzodiac.hardware.Pro775;

public final class Intake extends SubsystemBase implements ZmartDash {
    public final ZMotor convey = new Falcon(31);
    public final Pro775.Servo lift = new Pro775.Servo(32);
    private final DigitalInput toplimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(1);
    private State state = State.Standby;

    public Intake() {
        this.convey.set_pid(0.01, 0, 0);
        this.convey.profile.put("in", -20.0);
        this.convey.profile.put("out", 20.0);
        this.lift.set_pid(0.5, 5e-5, 0.01);
        this.lift.profile.put("down", 4000.0);
        this.lift.profile.put("up", 0.0);
        this.lift.profile.put("standby", 1500.0);
    }

    public Intake init() {
        this.convey.init();
        this.lift.init();
        return this;
    }

    public Intake standby() {
        // if (this.state == State.Standby)
        // return this;
        this.state = State.Standby;
        if ((this.lift.get() < this.lift.profile.get("standby") & this.bottomlimitSwitch.get()) || (this.lift.get() > this.lift.profile.get("standby") & this.toplimitSwitch.get())) {
            this.lift.go("standby");
        }
        this.convey.shutdown();
        this.debug("state", "standby");
        return this;
    }

    public Intake take() {
        // if (this.state == State.Taking)
        // return this;
        this.state = State.Taking;
        if (this.bottomlimitSwitch.get()) {
            this.lift.go("down");
        }
        this.convey.go("in");
        this.debug("state", "taking");
        return this;
    }

    public Intake send() {
        // if (this.state == State.Sending)
        // return this;
        this.state = State.Sending;
        if (this.toplimitSwitch.get()) {
            this.lift.go("up");
        }
        if (!this.toplimitSwitch.get()) {
            this.convey.go("out");
        } else {
            this.convey.shutdown();
        }
        this.debug("state", "sending");
        return this;
    }

    public Intake drop() {
        // if (this.state == State.Dropping)
        // return this;
        // this.state = State.Dropping;
        // this.lift.go("down");
        // this.convey.go("out");
        this.debug("state", "dropping");
        return this;
    }

    @Override
    public void periodic() {
    }

    @Override
    public String key() {
        return "Intake";
    }

    private enum State {
        Standby, Taking, Sending, Dropping,
    }
}
