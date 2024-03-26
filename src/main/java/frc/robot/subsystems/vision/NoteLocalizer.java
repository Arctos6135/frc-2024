package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

public class NoteLocalizer {
    private final Vision vision;
    private final Supplier<Pose2d> odometry;

    public NoteLocalizer(Vision vision, Supplier<Pose2d> odometry) {
        this.vision = vision;
        this.odometry = odometry;
    }
}
