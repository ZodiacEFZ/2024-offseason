package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.*;
import frc.libzodiac.hardware.Falcon;
import frc.libzodiac.hardware.TalonSRXMotor;
import frc.libzodiac.Constant;

public class Intake extends SubsystemBase implements ZmartDash {
    public final Falcon convey = new Falcon(31);
    public final TalonSRXMotor.Servo lift = new TalonSRXMotor.Servo(32);

    public Intake() {
        this.convey.set_pid(0, 0, 0);
        this.convey.profile.put("in", 0.2);
        this.convey.profile.put("out", -0.2);
        this.lift.set_pid(0.01, 0.01, 0.01);
        this.lift.profile.put("down", 0.0);
        this.lift.profile.put("up", -300.0/Constant.TALONSRX_ENCODER_UNIT);
        this.lift.profile.put("standby", -300.0/Constant.TALONSRX_ENCODER_UNIT);
    }

    public Intake init() {
        this.convey.init();
        this.lift.init();
        return this;
    }

    public Intake drop() {
        this.lift.go("down");
        Util.blocking_wait(500);
        this.convey.go("out");
        Util.blocking_wait(1000);
        this.convey.stop();
        this.lift.go("standby");
        return this;
    }

    public Intake reset() {
        this.lift.reset();
        return this;
    } 

    public ZCommand intake_ctrl(Zoystick ctrl) {
        return new Zambda<>((x) -> {
            final var trigger = ctrl.lTrigger();
            this.debug("rotate", x.lift.get());
            this.debug("intake", trigger > 0.3);
            if (trigger > 0.3) {
                x.lift.go("down");
                x.convey.go("in");
            } else {
                x.lift.go("standby");
                x.convey.stop();
            }
        }, this);
    }

    @Override
    public String key() {
        return "Intake";
    }
}

