package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static final double GEARBOX_RATIO = 1.0 / 170.0;

    // The arm length in meters, measured in CAD.
    public static final double ARM_LENGTH = Units.inchesToMeters(12.587);
    // In kg (complete guess)
    public static final double ARM_WEIGHT = 7;
    
    // Radians.
    public static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI * GEARBOX_RATIO;
    // Radians per second.
    public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI * GEARBOX_RATIO / 60.0;

    public static final double STARTING_POSITION = Units.degreesToRadians(-55);

    // TODO implement and tune the following :)
    public static final float MAX_POSITION = (float) Math.PI / 2; // Soft stop for max (farthest from floor) position
    public static final float MIN_POSITION = 90; // Soft stop for min (closest to floor) position
    
    public static final double SPEAKER_SCORING_POSITION = Units.degreesToRadians(-55);
    public static final double AMP_SCORING_POSITION = Units.degreesToRadians(-55);

    // Current limit for arm motor.
    public static final int CURRENT_LIMIT = 15;
}
