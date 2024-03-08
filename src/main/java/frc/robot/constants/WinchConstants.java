package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class WinchConstants {
    public static final int CURRENT_LIMIT = 20;

    public static final double GEARBOX_RATIO = 1.0 / 100.0;

    public static final double WINCH_PERIMETER = Units.inchesToMeters(1.0);

    public static final double POSITION_CONVERSION_FACTOR = WINCH_PERIMETER * GEARBOX_RATIO;

    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;
}
