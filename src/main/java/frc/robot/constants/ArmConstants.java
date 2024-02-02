package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    // TODO Incredible Zero GEARBOX_RATIO
    public static final double GEARBOX_RATIO = 0;

    // The arm length in meters.
    public static final double ARM_LENGTH = 0;
    // In kg
    public static final double ARM_WEIGHT = 0;
    
    // Undecided while design figures out what to do.
    public static final double ENCODER_CONVERSION_FACTOR = 2 * Math.PI * GEARBOX_RATIO;

    public static final double STARTING_POSITION = Units.degreesToRadians(-55);

    // TODO implement and tune the following :)
    public static final float MAX_POSITION = 90; // Soft stop for max (farthest from floor) position
    public static final float MIN_POSITION = 90; // Soft stop for min (closest to floor) position
    
    public static final double SPEAKER_SCORING_POSITION = Units.degreesToRadians(-55);
    public static final double AMP_SCORING_POSITION = Units.degreesToRadians(-55);

    // Current limit for arm motor.
    public static final int CURRENT_LIMIT = 15;
}
