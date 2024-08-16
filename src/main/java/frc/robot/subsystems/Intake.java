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
    private final DigitalInput toplimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(1);
    private State state = State.Standby;

    public Intake() {
        this.convey.set_pid(0.01, 0, 0);
        this.convey.profile.put("in", -20.0);
        this.convey.profile.put("out", 20.0);
        this.lift.set_pid(0.5, 5e-5, 50);
        this.lift.profile.put("down", 3850.0);
        this.lift.profile.put("up", 0.0);
        this.lift.profile.put("standby", 3900.0);
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
        if ((this.lift.get() < this.lift.profile.get("standby") & this.bottomlimitSwitch.get())
                || (this.lift.get() > this.lift.profile.get("standby") & this.toplimitSwitch.get())) {
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
            if (this.lift.get() < 5000)
                this.lift.go("down");
            else
                this.lift.shutdown();
        }
        this.convey.go("in");
        this.debug("state", "taking");
        return this;
    }

    public Intake send() {
        // if (this.state == State.Sending)
        // return this;
        this.state = State.Sending;
        // if (this.toplimitSwitch.get()) {
        // this.lift.go("up");
        // }
        // if (!this.toplimitSwitch.get()) {
        // this.convey.go("out");
        // } else {
        // this.convey.shutdown();
        // }
        if (this.lift.get() < 100)
            this.convey.go("out");
        this.debug("pos", this.lift.get());
        // if (this.lift.get() < 2000)
        // this.lift.shutdown();
        // else
        this.lift.go("up");
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
