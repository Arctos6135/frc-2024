package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PositionConstants {
    // Units are meters and degrees.
    public static final Pose2d RED_AMP = new Pose2d(16.54 - 0.79, 6.65, Rotation2d.fromDegrees(120));
    public static final Pose2d RED_STAGE = new Pose2d(16.54 - 1.35, 5.60, Rotation2d.fromDegrees(180));
    public static final Pose2d RED_SOURCE = new Pose2d(16.54 - 0.76, 4.40, Rotation2d.fromDegrees(-120));
    public static final Pose2d BLUE_AMP = new Pose2d(0.79, 6.65, Rotation2d.fromDegrees(60));
    public static final Pose2d BLUE_STAGE = new Pose2d(1.35, 5.60, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_SOURCE = new Pose2d(0.76, 4.40, Rotation2d.fromDegrees(-60));
}
