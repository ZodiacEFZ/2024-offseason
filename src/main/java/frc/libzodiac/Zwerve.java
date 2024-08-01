// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libzodiac;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.util.Vec2D;

/**
 * A highly implemented class for hopefully all types of swerve control.
 */
public abstract class Zwerve extends SubsystemBase implements ZmartDash {

    public final Vec2D shape;
    /**
     * Gyro.
     */
    public final ZGyro gyro;
    /**
     * Swerve modules of a rectangular chassis.
     * <p>
     * Suppose the robot heads the positive x direction,
     * relationship between indice and positions of modules are as follows:
     * <table>
     * <thead>
     * <tr>
     * <th>Galaxy</th>
     * <th>Index</th>
     * </tr>
     * </thead><tbody>
     * <tr>
     * <td>I</td>
     * <td>0</td>
     * </tr>
     * <tr>
     * <td>II</td>
     * <td>1</td>
     * </tr>
     * <tr>
     * <td>III</td>
     * <td>2</td>
     * </tr>
     * <tr>
     * <td>IV</td>
     * <td>3</td>
     * </tr>
     * </tbody>
     * </table>
     */
    public final Module[] module;
    public boolean headless = false;

    /**
     * Modifier timed at the output speed of the chassis.
     */
    public double output = 1;

    /**
     * Creates a new Zwerve.
     *
     * @param modules See <code>Zwerve.module</code>.
     * @param gyro    The gyro.
     * @param shape   Shape of the robot, <code>x</code> for length and <code>y</code> for width.
     */
    public Zwerve(Module[] modules, ZGyro gyro, Vec2D shape) {
        this.module = modules;
        this.gyro = gyro;
        this.shape = shape;
    }

    /**
     * Method to calculate the radius of the rectangular robot.
     */
    private double radius() {
        return this.shape.div(2).r();
    }

    /**
     * Get the absolute current direction of the robot.j
     */
    public double dir_curr() {
        return this.gyro.yaw();
    }

    /**
     * Get the direction adjustment applied under headless mode.
     */
    private double dir_fix() {
        if (this.gyro == null)
            return 0;
        return this.headless ? -this.dir_curr() : 0;
    }

    /**
     * Initialize all modules.
     */
    public Zwerve init() {
        for (Module i : this.module) {
            i.init();
        }
        this.gyro.init();
        return this.opt_init();
    }

    /**
     * Optional initializations you would like to automatically invoke.
     */
    protected abstract Zwerve opt_init();

    /**
     * Kinematics part from 6941.
     */
    @Deprecated
    public Zwerve go_previous(Vec2D velocity, double omega) {
        var x = velocity.x;
        var y = velocity.y;
        var l = this.shape.x;
        var w = this.shape.y;
        var r = this.radius();
        var a = y - omega * l / r;
        var b = y + omega * l / r;
        var c = x - omega * w / r;
        var d = x + omega * w / r;
        var v1 = new Vec2D.Polar(Math.hypot(b, c), Math.atan2(b, c));
        var v2 = new Vec2D.Polar(Math.hypot(b, d), Math.atan2(b, d));
        var v3 = new Vec2D.Polar(Math.hypot(a, d), Math.atan2(a, d));
        var v4 = new Vec2D.Polar(Math.hypot(a, c), Math.atan2(a, c));
        var max = Math.max(Math.max(v1.r, v2.r), Math.max(v3.r, v4.r));
        if (max > 1) {
            v1 = v1.div(max);
            v2 = v2.div(max);
            v3 = v3.div(max);
            v4 = v4.div(max);
        }
        this.module[0].go(v2.into());
        this.module[1].go(v3.into());
        this.module[2].go(v4.into());
        this.module[3].go(v1.into());
        return this;
    }

    /**
     * Kinematics part rewritten using vector calculations.
     */
    public Zwerve go(Vec2D velocity, double omega) {
        var l = this.shape.x / 2;
        var w = this.shape.y / 2;
        var vt = omega;
        velocity = velocity.rot(this.dir_fix());
        Vec2D[] v = {
                new Vec2D(-l, -w).with_r(vt).add(velocity),
                new Vec2D(l, -w).with_r(vt).add(velocity),
                new Vec2D(l, w).with_r(vt).add(velocity),
                new Vec2D(-l, w).with_r(vt).add(velocity),
        };
        var max = v[0].max(v[1]).max(v[2]).max(v[3]).r();
        if (max > 1) for (int i = 0; i < 4; i++) v[i] = v[i].div(max);
        this.module[0].go(v[0].mul(output));
        this.module[1].go(v[1].mul(output));
        this.module[2].go(v[2].mul(output));
        this.module[3].go(v[3].mul(output));
        return this;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Switch headless mode.
     */
    public Zwerve headless() {
        this.headless = !this.headless;
        return this;
    }

    /**
     * Enable/disable headless mode.
     */
    public Zwerve headless(boolean status) {
        this.headless = status;
        return this;
    }

    /**
     * Get the controlling <code>Command</code> for the current instance.
     */
    public Drive control(Zoystick vel, Zoystick rot) {
        return new Drive(this, vel, rot);
    }

    public ZCommand drive_forward() {
        return new ZLambda<>((x) -> x.go(new Vec2D(0.1, 0), 0), this);
    }

    public ZCommand single_joystick_drive(Zoystick zoystick) {
        return new ZLambda<>((x) -> {
            var vel = new Vec2D(-zoystick.ly(), -zoystick.lx());
            this.debug("vel", vel + "");
            this.debug("rot", zoystick.rx());
            x.go(vel, zoystick.rx());
        }, this);
    }

    public ZCommand check_headless(Zoystick zoystick) {
        return new ZLambda<>((x) -> {
            if (zoystick.button("X")) {
                this.headless();
            }
            this.debug("x", zoystick.button("X"));
            this.debug("headless", this.headless);
            this.debug("gyro", this.gyro.yaw());
        }, this);
    }

    public ZCommand check_wheel_reset(Zoystick zoystick) {
        return new ZLambda<>((x) -> {
            if (zoystick.button("A")) {
                this.reset();
            }
            if (zoystick.button("Y")) {
                this.module[0].clear();
                this.module[1].clear();
                this.module[2].clear();
                this.module[3].clear();
            }
        }, this);
    }

    public Zwerve reset() {
        this.module[0].reset();
        this.module[1].reset();
        this.module[2].reset();
        this.module[3].reset();
        this.gyro.reset();
        return this;
    }

    public Zwerve clear() {
        this.module[0].clear();
        this.module[1].clear();
        this.module[2].clear();
        this.module[3].clear();
        this.gyro.reset();
        return this;
    }

    @Override
    public String key() {
        return "Zwerve";
    }

    /**
     * Defines one swerve module.
     */
    public interface Module {

        Module init();

        Module go(Vec2D velocity);

        Module reset();

        Module clear();
    }

    /**
     * The driving <code>Command</code> for the chassis.
     */
    public static class Drive extends ZCommand {

        /**
         * The chassis.
         */
        private final Zwerve chassis;
        /**
         * The joystick that controls translation.
         */
        private final Zoystick vel_ctrl;

        /**
         * The joystick that controls rotation.
         */
        private final Zoystick rot_ctrl;

        private boolean inv_vx = false;
        private boolean inv_vy = false;
        private boolean inv_rot = false;

        /**
         * Creates a new Drive.
         */
        public Drive(Zwerve chassis, Zoystick vel, Zoystick rot) {
            // Use addRequirements() here to declare subsystem dependencies.
            this.chassis = this.require(chassis);
            this.vel_ctrl = vel;
            this.rot_ctrl = rot;
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        protected Drive exec() {
            var v_x = this.vel_ctrl.lx();
            var v_y = this.vel_ctrl.ly();
            var omega = this.rot_ctrl.getRawAxis(0);
            if (this.inv_vx)
                v_x = -v_x;
            if (this.inv_vy)
                v_y = -v_y;
            if (this.inv_rot)
                omega = -omega;
            this.chassis.go(new Vec2D(v_x, v_y), omega);
            return this;
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }

        /**
         * Configure whether to invert the output of the chassis.
         */
        public Drive inv(boolean v_x, boolean v_y, boolean rot) {
            this.inv_vx = v_x;
            this.inv_vy = v_y;
            this.inv_rot = rot;
            return this;
        }
    }
}
