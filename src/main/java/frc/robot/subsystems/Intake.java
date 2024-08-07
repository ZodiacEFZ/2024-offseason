package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ZCommand;
import frc.libzodiac.Zambda;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.Zoystick;
import frc.libzodiac.hardware.Falcon;
import frc.libzodiac.hardware.TalonSRXMotor;

public class Intake extends SubsystemBase implements ZmartDash {
    public Falcon intakeMotor;
    public TalonSRXMotor.Servo rotateMotor;

    public Intake(int intakeMotor, int rotateMotor) {
        this.intakeMotor = new Falcon(intakeMotor);
        this.intakeMotor.set_pid(0, 0, 0);
        this.intakeMotor.init();
        this.rotateMotor = new TalonSRXMotor.Servo(rotateMotor);
        this.rotateMotor.set_pid(0.1, 0.1, 0.1);
        this.rotateMotor.init();
    }

    public Intake output(double speed) {
        this.debug("output", speed);
        this.intakeMotor.go(-speed);
        return this;
    }

    public ZCommand intake(Zoystick zoystick) {
        return new Zambda<>((x) -> {
            var trigger = zoystick.lTrigger();
            this.debug("rotate", rotateMotor.get());
            this.debug("intake", trigger > 0.3);
            if (trigger > 0.3) {
                rotateMotor.go(0);
                intakeMotor.go(0.2);
            } else {
                rotateMotor.go(-200);
                intakeMotor.stop();
            }
        }, this);
    }

    @Override
    public String key() {
        return "Intake";
    }
}

