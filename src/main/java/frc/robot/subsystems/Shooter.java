package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.ZCommand;
import frc.libzodiac.ZLambda;
import frc.libzodiac.ZmartDash;
import frc.libzodiac.Zoystick;

public class Shooter extends SubsystemBase implements ZmartDash {

    public TalonFX shooter1;

    public Shooter(int id1, boolean inverted1) {
        shooter1 = new TalonFX(id1);
        shooter1.setInverted(inverted1);
    }

    public ZCommand shoot(double speed) {
        return new ZLambda<>((x) -> {
            this.debug("shooter", speed);
            this.shooter1.set(speed);
        }, this);
    }

    @Override
    public String key() {
        return "Shooter";
    }
}

