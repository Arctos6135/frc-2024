package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PositionConstants {
    // Units are meters and degrees.
    public static final Pose2d RED_AMP = new Pose2d(15.70, 6.65, Rotation2d.fromDegrees(120));
    public static final Pose2d RED_STAGE = new Pose2d(15.07, 5.58, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_SOURCE = new Pose2d(15.70, 4.30, Rotation2d.fromDegrees(-120));
    public static final Pose2d BLUE_AMP = new Pose2d(0.79, 6.65, Rotation2d.fromDegrees(60));
    public static final Pose2d BLUE_STAGE = new Pose2d(1.38, 5.58, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_SOURCE = new Pose2d(0.79, 4.30, Rotation2d.fromDegrees(-60));
}
