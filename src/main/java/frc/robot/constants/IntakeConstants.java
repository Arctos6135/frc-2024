package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.Intake;

public class IntakeConstants { // these numbers are copied from drive constants and are temporary
    public static double GEARBOX_RATIO = 1.0 / 5.0;
    public static double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(1.8) * Math.PI;
    public static double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;

    // In kg / square meter.
    public static double MOMENT_OF_INERTIA = 0.0204847757;

    public static int MEDIAN_FILTER_SIZE = 12;

    // The voltage at which the intake motors should rotate at when intaking
    public static double VOLTAGE = 12;

    public static double INTAKE_CURRENT = 18;

    // Current limit of the intake motor.
    public static int CURRENT_LIMIT = 20;

    // Constants for Feed Command
    public static double FEED_VOLTAGE = 6;
    public static double DISTANCE_TOLERANCE = 0.3;
    public static double EMPTY_CURRENT; // set to a default value
    public static double STANDARD_CURRENT_ERROR = 1;

    public static double REVERSE_DISTANCE = Units.inchesToMeters(-6);


    public static void setEmptyCurrent(Intake intake) { // maybe not needed
        intake.setVoltage(FEED_VOLTAGE);
        EMPTY_CURRENT = intake.getFilteredCurrent();
    }
}
