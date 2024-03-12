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

    // Angles are measured from when the short part of the arm is horizontal, in radians, with negative numbers meaning the arm is below the horizontal.

    public static final double STARTING_POSITION = Units.degreesToRadians(6);
    public static final float MAX_POSITION = (float) Units.degreesToRadians(60); // Soft stop for max (farthest from floor) position
    public static final float MIN_POSITION = (float) Units.degreesToRadians(6); // Soft stop for min (closest to floor) position
    
    public static final double AMP_SCORING_POSITION = STARTING_POSITION + 1.2; // Binded to X
    public static final double SPEAKER_SCORING_POSITION = STARTING_POSITION; // Binded to Y

    public static final double ARM_TOLERANCE = 5 * Math.PI / 180;

    // Current limit for arm motor.
    public static final int CURRENT_LIMIT = 30;
}
