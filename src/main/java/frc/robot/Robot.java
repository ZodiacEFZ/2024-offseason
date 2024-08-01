// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private static RobotContainer m_bot;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_bot = new RobotContainer().init();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
//        var scheduler = CommandScheduler.getInstance();
//        scheduler.schedule(m_bot.chassis.drive_forward());
//        scheduler.run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // m_autonomousCommand = RobotContainer.getAutoCommand();

        // // schedule the autonomous command (example)
        // if (m_autonomousCommand != null) {
        // m_autonomousCommand.schedule();
        // }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // if (m_autonomousCommand != null) {
        // m_autonomousCommand.cancel();
        // }
        // if (RobotContainer.getTeleopShooterCommand() != null)
        // RobotContainer.getTeleopShooterCommand().schedule();
        // if (RobotContainer.getTeleopIntakeCommand() != null)
        // RobotContainer.getTeleopIntakeCommand().schedule();
        m_bot.chassis.reset();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        // CommandScheduler.getInstance().schedule(RobotContainer.swerveDrive);
        // CommandScheduler.getInstance().schedule(RobotContainer.sc);
        var scheduler = CommandScheduler.getInstance();
        scheduler.schedule(m_bot.chassis.single_joystick_drive(m_bot.drive));
        scheduler.run();
        scheduler.schedule(m_bot.shooter.shoot(m_bot.drive));
        scheduler.run();
        scheduler.schedule(m_bot.chassis.check_headless(m_bot.drive));
        scheduler.run();
        scheduler.schedule(m_bot.chassis.check_wheel_reset(m_bot.drive));
        scheduler.run();
//        scheduler.schedule(m_bot.chassis_ctrl());
//        scheduler.schedule(new RunCommand(() -> {
//            m_bot.shooter1.set(0.1);
//            m_bot.shooter2.set(0.1);
//        }));
//        scheduler.run();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        m_bot.chassis.clear();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        var scheduler = CommandScheduler.getInstance();
        scheduler.schedule(m_bot.chassis.drive_forward());
        scheduler.run();
    }

    /**
     *
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
