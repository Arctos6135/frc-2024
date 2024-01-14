package frc.robot.constants;

public class DriveConstants {
    public static double GEARBOX_RATIO = 1 / 8.45; 
    public static double WHEEL_DIAMETER = 6.0; 
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; 
    public static double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
}