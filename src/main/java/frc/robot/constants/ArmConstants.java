package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    // TODO Incredible Zero GEARBOX_RATIO
    public static final double GEARBOX_RATIO = 1;

    // The arm length in meters.
    public static final double ARM_LENGTH = 1;
    // In kg
    public static final double ARM_WEIGHT = 1;
    
    // Undecided while design figures out what to do.
    public static final double ENCODER_CONVERSION_FACTOR = 2 * Math.PI * GEARBOX_RATIO;

    public static final double STARTING_POSITION = Units.degreesToRadians(-55);

    // TODO implement and tune the following :)
    // There may be some explosions with these large soft stops... o7
    public static final float MAX_POSITION = (float) Units.degreesToRadians(90); // Soft stop for max (farthest from floor) position
    public static final float MIN_POSITION = (float) Units.degreesToRadians(-90); // Soft stop for min (closest to floor) position
    
    public static final double AMP_SCORING_POSITION = Units.degreesToRadians(-55); // Binded to X
    public static final double SPEAKER_SCORING_POSITION = Units.degreesToRadians(-55); // Binded to Y

    public static final double ARM_TOLERANCE = 5 * Math.PI / 180;

    // Current limit for arm motor.
    public static final int CURRENT_LIMIT = 15;
}
