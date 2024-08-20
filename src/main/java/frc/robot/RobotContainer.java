// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.libzodiac.Zambda;
import frc.libzodiac.Zamera;
import frc.libzodiac.ui.Axis;
import frc.libzodiac.ui.Xbox;
import frc.robot.commands.Auto;
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
    public final Chassis chassis = new Chassis();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Zamera camera = new Zamera();
    public final Xbox driver = new Xbox(0);
    public final Xbox controller = new Xbox(1);

    public final Auto auto = new Auto(this.chassis, this.intake, this.shooter, Auto.Position.Center);

    public final Command drive = chassis.drive(driver.ly().inverted().map(Axis.ATAN_FILTER).threshold(.02), driver.lx().inverted().map(Axis.ATAN_FILTER).threshold(.02), driver.rx().inverted().threshold(.02));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        this.driver.a().on_press(new Zambda(this.chassis, this.chassis::toggle_headless));
        this.driver.b().on_press(new Zambda(this.chassis, this.chassis::reset_headless));
        this.driver.x().on_press(new Zambda(this.chassis, this.chassis::mod_reset));

        this.controller.a().on_down(new Zambda(this.intake, this.intake::amp)).on_release(new Zambda(this.intake, () -> this.intake.amp(false)));
        this.controller.lb().on_press(new Zambda(this.intake, this.intake.lift::reset));
        this.controller.lt().into().on_down(new Zambda(this.intake, this.intake::take)).on_release(new Zambda(this.intake, this.intake::standby));
        this.controller.rt().into().on_down(new Zambda(this.shooter, this.shooter::shoot)).on_down(new Zambda(this.intake, this.intake::send)).on_release(new Zambda(this.intake, this.intake::standby)).on_release(new Zambda(this.shooter, this.shooter::standby));
    }

    public RobotContainer init() {
        chassis.init();
        intake.init();
        shooter.init();
        camera.start();
        return this;
    }
}
