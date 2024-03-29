package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.VisionConstants;

public class VisionIOSim extends VisionIO {
    private final Translation2d notePosition;
    private final Supplier<Pose2d> positionSupplier;

    public VisionIOSim(Translation2d notePosition, Supplier<Pose2d> positionSupplier) {
        this.notePosition = notePosition;
        this.positionSupplier = positionSupplier;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.hasTarget = true;
        inputs.noteDistance = getNoteDistance();
        inputs.noteAngle = getNoteAngle();
    }

    public double getNoteDistance() {
        return this.notePosition.getDistance(positionSupplier.get().getTranslation());
    }

    public double getNoteAngle() {
        final Pose2d currentPose = positionSupplier.get();
        Logger.recordOutput("Vision/Target Pose", notePosition);
        final Translation2d robotRelativeNotePosition = notePosition.minus(currentPose.getTranslation());

        Rotation2d noteHeading = robotRelativeNotePosition.getAngle();
        Logger.recordOutput("Vision/Note Heading", noteHeading);
        //final double noteHeading = Math.atan2(robotRelativeNotePosition.getY(), robotRelativeNotePosition.getX());
        return noteHeading.minus(currentPose.getRotation()).getRadians();
    }
}