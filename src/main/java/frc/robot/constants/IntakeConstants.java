package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class IntakeConstants { // these numbers are copied from drive constants and are temporary
    public static double GEARBOX_RATIO = 1 / 5;
    public static double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(0.85) * 2 * Math.PI;
    public static double ENCODER_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static int MEDIAN_FILTER_SIZE = 14;

    // The voltage at which the intake motors should rotate at when intaking
    public static double VOLTAGE = 12;

    public static double MAX_CURRENT = 14;
}
