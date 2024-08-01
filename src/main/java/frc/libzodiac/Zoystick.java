package frc.libzodiac;

import edu.wpi.first.wpilibj.Joystick;

import java.util.HashMap;
import java.util.HashSet;
import java.util.function.Function;

/**
 * Inheritd from WPILib's <code>Joystick</code>,
 * added some utilities to enhance using experience.
 */
public class Zoystick extends Joystick {

    /**
     * Filters the input value as a parabola, e.g. -0.5 -> -0.25.
     */
    public static final Function<Double, Double> quad_filter = (x) -> x / Math.abs(x) * x * x;
    /**
     * Button names to button IDs mappings.
     */
    public HashMap<String, Integer> key_map = new HashMap<>();
    /**
     * Defines the collection of axis to invert.
     */
    public HashSet<Integer> inv = new HashSet<>();
    /**
     * Whether to invert x-axis.
     */
    public boolean inv_x = false;
    /**
     * Whether to invert y-axis.
     */
    public boolean inv_y = false;
    /**
     * Function to pre-process the input value of joystick.
     */
    public Function<Double, Double> filter_fn = (x) -> x;

    public Zoystick(int port) {
        super(port);
    }

    /**
     * Filters the input value with a threshold, inputs with absolute values less
     * than it will be ignored.
     */
    public static Function<Double, Double> thre_filter(double thre) {
        Function<Double, Double> lambda = (x) -> {
            if (Math.abs(x) > thre) {
                return x / Math.abs(x) * x * x;
            }
            return 0.0;
        };
        return lambda;
    }

    public static Function<Double, Double> default_filter(double thre) {
        Function<Double, Double> lambda = (x) -> {
            if (Math.abs(x) > thre) {
                return x;
            }
            return 0.0;
        };
        return lambda;
    }

    /**
     * Set the output invert status of an axis.
     */
    public Zoystick inv(int axis, boolean inv) {
        if (inv)
            this.inv.add(axis);
        else
            this.inv.remove(axis);
        return this;
    }

    /**
     * Set an axis to be inverted.
     */
    public Zoystick inv(int axis) {
        return this.inv(axis, true);
    }

    /**
     * Get the filtered input of an axis.
     */
    public double axis(int axis) {
        var a = this.filter_fn.apply(this.getRawAxis(axis));
        if (this.inv.contains(axis))
            a = -a;
        return a;
    }

    /**
     * Gets the x-axis of the left joystick.
     */
    public double lx() {
        return this.axis(0);
    }

    /**
     * Gets the y-axis of the left joystick.
     */
    public double ly() {
        return this.axis(1);
    }

    /**
     * Gets the x-axis of the right joystick.
     */
    public double rx() {
        return this.axis(4);
    }

    /**
     * Gets the axis of the left joystick.
     */
    public double lTrigger() {
        return this.axis(2);
    }

    /**
     * Gets the axis of the right joystick.
     */
    public double rTrigger() {
        return this.axis(3);
    }

    /**
     * Set the input filter function.
     */
    public Zoystick set_filter(Function<Double, Double> f) {
        this.filter_fn = f;
        return this;
    }

    /**
     * Gives the specified button ID a name for future visits.
     */
    public Zoystick map(int button, String name) {
        this.key_map.put(name, button);
        return this;
    }

    /**
     * Get the button status with the specified name.
     */
    public boolean button(String button) {
        return this.getRawButton(this.key_map.get(button));
    }

    /**
     * Similar to <code>button</code>,
     * but returns true only if the button is pressed from the release status since
     * last call.
     */
    public boolean button_pressed(String button) {
        return this.getRawButtonPressed(this.key_map.get(button));
    }

    /**
     * Similar to <code>button</code>,
     * but returns true only if the button is released from pressed status since
     * last call.
     */
    public boolean button_released(String button) {
        return this.getRawButtonReleased(this.key_map.get(button));
    }
}
