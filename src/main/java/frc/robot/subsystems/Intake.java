package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.Falcon;
import frc.libzodiac.hardware.Pro775;
import frc.libzodiac.Constant;
import frc.libzodiac.ZmartDash;

public final class Intake extends SubsystemBase implements ZmartDash {
    public final Falcon convey = new Falcon(31);
    public final Pro775.Servo lift = new Pro775.Servo(32);

    public Intake() {
        this.convey.set_pid(0, 0, 0);
        this.convey.profile.put("in", 0.2);
        this.convey.profile.put("out", -0.2);
        this.lift.set_pid(0.01, 0.01, 0.01);
        this.lift.profile.put("down", 0.0);
        this.lift.profile.put("up", -300.0 / Constant.TALONSRX_ENCODER_UNIT);
        this.lift.profile.put("standby", -300.0 / Constant.TALONSRX_ENCODER_UNIT);
    }

    public Intake init() {
        this.convey.init();
        this.lift.init();
        return this;
    }

    private enum State {
        Standby,
        Taking,
        Sending,
        Dropping,
    }

    private State state = State.Standby;

    public Intake reset() {
        this.lift.reset();
        return this;
    }

    public Intake standby() {
        if (this.state == State.Standby)
            return this;
        this.state = State.Standby;
        this.lift.go(-200);
        this.convey.shutdown();
        return this;
    }

    public Intake take() {
        if (this.state == State.Taking)
            return this;
        this.state = State.Taking;
        this.lift.go(0);
        this.convey.go(-1);
        return this;
    }

    public Intake send() {
        if (this.state == State.Sending)
            return this;
        this.state = State.Sending;
        this.lift.go(-500);
        this.convey.go(1);
        return this;
    }

    public Intake drop() {
        if (this.state == State.Dropping)
            return this;
        this.state = State.Dropping;
        this.lift.go(0);
        this.convey.go(1);
        return this;
    }

    @Override
    public void periodic() {
        return;
    }

    @Override
    public String key() {
        return "Intake";
    }
}
