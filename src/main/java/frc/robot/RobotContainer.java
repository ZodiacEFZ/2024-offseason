// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.libzodiac.Zambda;
import frc.libzodiac.Zoystick;
import frc.libzodiac.Zwerve;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public Chassis chassis = new Chassis();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();

    public Zoystick drive = new Zoystick(0)
            .map(1, "A")
            .map(2, "B")
            .map(3, "X")
            .map(4, "Y")
            .bind("X", new Zambda<>(Zwerve::toggle_headless, chassis))
            .set_filter(Zoystick.default_filter(0.08));

    public Zoystick ctrl = new Zoystick(1)
            .map(1, "A")
            .map(2, "B")
            .map(3, "X")
            .map(4, "Y");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    public RobotContainer init() {
        chassis.init();
        chassis.reset();
        intake.init();
        shooter.init();
        return this;
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        final var ctrl = new CommandXboxController(0);
        ctrl.a().toggleOnTrue(new Zambda<>(Intake::drop, this.intake));
        ctrl.a().toggleOnFalse(new Zambda<>(Intake::standby, this.intake));
        ctrl.leftTrigger().toggleOnTrue(new Zambda<>(Intake::take, this.intake));
        ctrl.leftTrigger().toggleOnFalse(new Zambda<>(Intake::standby, this.intake));
        ctrl.rightTrigger().toggleOnTrue(new Zambda<>(Shooter::shoot, this.shooter));
        ctrl.rightTrigger().toggleOnTrue(new Zambda<>(Intake::send, this.intake));
        ctrl.rightTrigger().toggleOnFalse(new Zambda<>(Intake::standby, this.intake));
        ctrl.rightTrigger().toggleOnFalse(new Zambda<>(Shooter::standby, this.shooter));
    }

}
