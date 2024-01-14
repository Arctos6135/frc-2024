package frc.robot.constants;

public class DriveConstants {
    private static double precisionFwdRev = 0.6;
    private static double precisionTurn = 0.7;

    public static double GEARBOX_RATIO = 1 / 8.45; 
    public static double WHEEL_DIAMETER = 6.0; 
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; 
    public static double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static double CONTROLLER_DEADZONE = 0.15;
}