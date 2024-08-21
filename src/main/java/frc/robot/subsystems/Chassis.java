package frc.robot.subsystems;

import frc.libzodiac.InertialNavigation;
import frc.libzodiac.Zwerve;
import frc.libzodiac.hardware.Falcon;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.Pigeon;
import frc.libzodiac.hardware.group.FalconSwerve;
import frc.libzodiac.util.PIDProfile;
import frc.libzodiac.util.Vec2D;

public class Chassis extends Zwerve {
    public static final Falcon[] speed = { new Falcon(5), new Falcon(6).invert(), new Falcon(7).invert(), new Falcon(8) };

    public static final Falcon.Servo[] angle = { new Falcon.Servo(1).invert(), new Falcon.Servo(2).invert(), new Falcon.Servo(3).invert(), new Falcon.Servo(4) };

    public static final MagEncoder[] encoder = { new MagEncoder(9), new MagEncoder(10), new MagEncoder(11), new MagEncoder(12) };

    private static final FalconSwerve[] mods = { new FalconSwerve(speed[0], angle[0], encoder[0]), new FalconSwerve(speed[1], angle[1], encoder[1]), new FalconSwerve(speed[2], angle[2], encoder[2]), new FalconSwerve(speed[3], angle[3], encoder[3]) };

    private static final Pigeon gyro = new Pigeon(0);

    public static final InertialNavigation inav = new InertialNavigation(gyro);

    /**
     * Creates a new Chassis.
     */
    public Chassis() {
        super(mods, gyro.yaw(), new Vec2D(114, 114));
        super.output = 600;
        final var v = new PIDProfile(0.1, 5, 0);
        final var a = new PIDProfile(0.3, 0, 0);
        // Mod I.
        mods[0].speed_motor.set_pid(v);
        mods[0].angle_motor.set_pid(a);
        // Mod II.
        mods[1].speed_motor.set_pid(v);
        mods[1].angle_motor.set_pid(a);
        // Mod III.
        mods[2].speed_motor.set_pid(v);
        mods[2].angle_motor.set_pid(a);
        // Mod IV.
        mods[3].speed_motor.set_pid(v);
        mods[3].angle_motor.set_pid(a);
    }

    @Override
    protected Zwerve opt_init() {
        gyro.init();
        inav.init();
        encoder[0].set_zero(3581); //todo
        encoder[1].set_zero(408);
        encoder[2].set_zero(149);
        encoder[3].set_zero(1921);
        this.mod_reset();
        return this;
    }

    public void reset_headless() {
        super.headless_zero = gyro.get("yaw");
        this.headless = false;
    }
}