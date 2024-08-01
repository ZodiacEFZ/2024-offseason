package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ZCommand;
import frc.libzodiac.ZLambda;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.Zoystick;

public class Shooter extends SubsystemBase implements ZmartDash {

    public TalonFX shooter1;
    public TalonFX shooter2;

    public Shooter(int id1, int id2, boolean inverted1, boolean inverted2) {
        shooter1 = new TalonFX(id1);
        shooter2 = new TalonFX(id2);
        shooter1.setInverted(inverted1);
        shooter2.setInverted(inverted2);
    }

    public ZCommand shoot(Zoystick zoystick) {
        return new ZLambda<>((x) -> {
            var speed = zoystick.rTrigger();
            this.debug("shooter", speed);
            this.shooter1.set(speed);
            this.shooter2.set(speed);
        }, this);
    }

    @Override
    public String key() {
        return "Shooter";
    }
}

