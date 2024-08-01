// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.libzodiac.Zoystick;
import frc.robot.subsystems.Chassis;
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

    public Shooter shooter = new Shooter(18, 30, true, false);

    public Zoystick drive = new Zoystick(0)
            .map(1, "A")
            .map(2, "B")
            .map(3, "X")
            .map(4, "Y")
            .set_filter(Zoystick.default_filter(0.08));
//    public Zoystick ctrl = new Zoystick(1)
//            .map(0, "X")
//            .map(1, "Y")
//            .set_filter(Zoystick.thre_filter(0.1));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

//    public Command chassis_ctrl() {
//        return chassis
//                .control(drive, ctrl)
//                .inv(false, false, false);
//    }

    public RobotContainer init() {
        chassis.init();
        chassis.reset();
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
//        new CommandXboxController(0).x().toggleOnTrue(new ZLambda<Zwerve>(Zwerve::headless, this.chassis));
    }

}
