package frc.robot.constants;

public class DriveConstants {
    public static int DRIVER_CONTROLLER = 0;
    public static int OPERATOR_CONTROLLER = 1;

    public static double PRECISION_FWD_REV = 0.6;
    public static double PRECISION_TURN = 0.7;

    public static double GEARBOX_RATIO = 1 / 8.45; 
    public static double WHEEL_DIAMETER = 6.0; 
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; 
    public static double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static double CONTROLLER_DEADZONE = 0.15;
}