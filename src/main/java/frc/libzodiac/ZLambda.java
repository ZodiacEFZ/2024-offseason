package frc.libzodiac;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;

/**
 * Allows you to construct wpilib's <code>Command</code> with a lambda.
 */
public class ZLambda<T extends Subsystem> extends ZCommand {
    public final T src;
    public final Consumer<T> exec;

    public ZLambda(Consumer<T> exec, T src) {
        this.exec = exec;
        this.src = src;
        addRequirements(src);
    }

    @Override
    protected ZCommand exec() {
        this.exec.accept(this.src);
        return this;
    }
}