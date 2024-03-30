package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    // these numbers are copied from drive constants and are temporary
    public static final double GEARBOX_RATIO = 1;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; 
    public static final double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO / 60.0;

    public static final double CONTROLLER_DEADZONE = 0.15;

    // This is a placeholder, need to be tuned.
    public static final double SPEAKER_RPS = 20;
    public static final double AMP_RPS = 20;
    public static final double FERRY_RPS = 10; // random choice, needs tuning.

    // Limits current through motor to limit brownouts.
    public static final int CURRENT_LIMIT = 50;

    // These constants need to be tuned.
    // Static gain in volts (volts needed to make the flywheel barely move).
    public static final double kS = 0;
    // Velocity gain in volts/ radians per second (voltage required to "cruise" at a given constant velocity).
    public static final double kV = 0.0019 * 60 * 3.8;
    // Acceleration gain in volts/ radians per second^2.
    public static final double kA = 0.001;

    // Kg / m^2
    public static final double MOMENT_OF_INERTIA = 0.07137412;

    public static final double FEED_RPS = 25;

    public static final double FEED_VOLTAGE = 0.5;

    // The voltage we run the shooter at for the advance shooter command
    public static final double ADVANCE_VOLTAGE = 6;

    // Meters to run the shooter tread forward
    public static final double ADVANCE_DISTANCE = 12;
}
