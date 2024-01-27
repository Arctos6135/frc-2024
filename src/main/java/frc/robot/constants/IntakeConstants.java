package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class IntakeConstants { // these numbers are copied from drive constants and are temporary
    public static double GEARBOX_RATIO = 1 / 8.45;
    public static double WHEEL_DIAMETER = Units.inchesToMeters(6);
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; 
    public static double ENCODER_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static double CONTROLLER_DEADZONE = 0.15;
    public static int MEDIAN_FILTER_SIZE = 14;

    // The voltage at which the intake motors should rotate at when intaking
    public static double VOLTAGE = 14;

    public static double MAX_CURRENT = 14;
}
