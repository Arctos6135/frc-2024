package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    // TODO Incredible Zero GEARBOX_RATIO
    public static final double GEARBOX_RATIO = 0;
    
    // Undecided while design figures out what to do.
    public static final double ENCODER_CONVERSION_FACTOR = 2 * Math.PI * GEARBOX_RATIO;

    public static final double STARTING_POSITION = Units.degreesToRadians(-55);

    // Current limit for arm motor.
    public static final int CURRENT_LIMIT = 15;
}
