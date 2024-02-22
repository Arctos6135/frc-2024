package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    // these numbers are copied from drive constants and are temporary
    public static final double GEARBOX_RATIO = 1;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; 
    public static final double ENCODER_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static final double CONTROLLER_DEADZONE = 0.15;

    // This is a placeholder, need to be tuned.
    public static final double SPEAKER_RPS = 20;
    public static final double AMP_RPS = 15;

    // Limits current through motor to limit brownouts.
    public static final int CURRENT_LIMIT = 40;

    // These constants need to be tuned.
    // Static gain in volts (volts needed to make the flywheel barely move).
    public static final double kS = 0;
    // Velocity gain in volts/ rotations per second (voltage required to "cruise" at a given constant velocity).
    public static final double kV = 1;
    // Acceleration gain in volts/ rotations per second^2.
    public static final double kA = 1;

    public static final double FEED_RPS = 2;

    
}
