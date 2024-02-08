package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Constants related to the dimensioning and control of the drivetrain. Units are in meters, kilograms, seconds.
 */
public class DriveConstants {
    public static double PRECISION_FWD_REV = 0.1;
    public static double PRECISION_TURN = 0.1;

    public static double GEARBOX_RATIO = 1 / 8.45;
    public static double WHEEL_DIAMETER = Units.inchesToMeters(6);
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; 
    public static double ENCODER_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static double CONTROLLER_DEADZONE = 0.15;

    // Needs to be tuned :)
    public static final double ROTATION_TOLERANCE = 0.5;

    public static final double MAX_TELEOP_SPEED = 1.5;

    public static final double MAX_TURN_SPEED_FACTOR = 1;

    // Current limits for drivetrain motors.
    public static final int CURRENT_LIMIT = 30;
}