package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static String cameraName = "limelight";
    public static double cameraHeight = Units.inchesToMeters(15);
    public static double targetHeight = Units.inchesToMeters(1.33); // actual value
    public static double cameraPitch = Units.degreesToRadians(-25);
}