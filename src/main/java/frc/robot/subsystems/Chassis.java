// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.libzodiac.ZEncoder;
import frc.libzodiac.Zwerve;
import frc.libzodiac.hardware.Falcon;
import frc.libzodiac.hardware.Pigeon;
import frc.libzodiac.hardware.TalonSRXEncoder;
import frc.libzodiac.hardware.group.FalconSwerve;
import frc.libzodiac.hardware.group.FalconWithEncoder;
import frc.libzodiac.util.PIDProfile;
import frc.libzodiac.util.Vec2D;

public class Chassis extends Zwerve {

    public static final Falcon[] speed = {
            new Falcon(10),
            new Falcon(11),
            new Falcon(8),
            new Falcon(5)
    };

    public static final Falcon.Servo[] servo = {
            new Falcon.Servo(3),
            new Falcon.Servo(2),
            new Falcon.Servo(12),
            new Falcon.Servo(6),
    };

    public static final ZEncoder[] encoder = {
            new TalonSRXEncoder(1),
            new TalonSRXEncoder(4),
            new TalonSRXEncoder(7),
            new TalonSRXEncoder(9),
    };

    public static final FalconWithEncoder[] angle = {
            new FalconWithEncoder(servo[0], encoder[0]),
            new FalconWithEncoder(servo[1], encoder[1]),
            new FalconWithEncoder(servo[2], encoder[2]),
            new FalconWithEncoder(servo[3], encoder[3]),
    };

    private static final FalconSwerve[] mods = {
            new FalconSwerve(speed[0], angle[0], new Vec2D(1, 1).mul(0.25)),
            new FalconSwerve(speed[1], angle[1], new Vec2D(1, -1).mul(0.25)),
            new FalconSwerve(speed[2], angle[2], new Vec2D(-1, 1).mul(0.25)),
            new FalconSwerve(speed[3], angle[3], new Vec2D(1, -1).mul(0.25)),
    };

    private static final Pigeon gyro = new Pigeon(0);

    /**
     * Creates a new Chassis.
     */
    public Chassis() {
        super(mods, gyro, new Vec2D(114, 114));
        var v = new PIDProfile(0.15, 0, 0);
        var a = new PIDProfile(0.05, 0.001, 0);
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
    public void periodic() {
        // This method will be called once per scheduler run
        super.periodic();
    }

    @Override
    protected Zwerve opt_init() {
        super.gyro.zero_yaw = super.gyro.yaw();
        return this;
    }
}
