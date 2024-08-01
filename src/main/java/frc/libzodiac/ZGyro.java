package frc.libzodiac;

public abstract class ZGyro {
    public final int can_id;

    /**
     * Zero position in raw sensor units.
     */
    public double zero_yaw = 0;
    /**
     * Zero position in raw sensor units.
     */
    public double zero_pitch = 0;
    /**
     * Zero position in raw sensor units.
     */
    public double zero_roll = 0;

    public ZGyro(int can_id) {
        this.can_id = can_id;
    }

    public abstract ZGyro init();

    protected abstract double get_yaw();

    protected abstract double get_pitch();

    protected abstract double get_roll();

    public abstract ZGyro reset();

    /**
     * Read current yaw from sensor.
     */
    public double yaw() {
        return this.get_yaw() - this.zero_yaw;
    }

    /**
     * Read current pitch from sensor.
     */
    public double pitch() {
        return this.get_pitch() - this.zero_pitch;
    }

    /**
     * Read current roll from sensor.
     */
    public double roll() {
        return this.get_roll() - this.zero_roll;
    }
}
