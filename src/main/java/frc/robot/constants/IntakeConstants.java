package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.Intake;

public class IntakeConstants { // these numbers are copied from drive constants and are temporary
    public static double GEARBOX_RATIO = 1 / 5;
    public static double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(3) * Math.PI;
    public static double ENCODER_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static int MEDIAN_FILTER_SIZE = 14;

    // The voltage at which the intake motors should rotate at when intaking
    public static double VOLTAGE = 1;

    public static double MAX_EMPTY_CURRENT = 14;

    // Current limit of the intake motor.
    public static int CURRENT_LIMIT = 20;

    // Constants for Feed Command
    public static double FEED_VOLTAGE = 6;
    public static double DISTANCE_TOLERANCE = 0.3;
    public static double EMPTY_CURRENT; // set to a default value
    public static double STANDARD_CURRENT_ERROR = 1;


    public static void setEmptyCurrent(Intake intake) { // maybe not needed
        intake.setVoltage(FEED_VOLTAGE);
        EMPTY_CURRENT = intake.getFilteredCurrent();
    }
}
