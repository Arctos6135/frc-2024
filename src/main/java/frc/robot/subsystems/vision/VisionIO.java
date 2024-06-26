package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.VisionConstants;

public class VisionIO {
    @AutoLog
    public static class VisionInputs {
        boolean hasTarget;
        boolean isConnected;
        double noteDistance;
        double noteAngle;
        double noteArea;
        double notePitch;
        double latencyMillis;
    }

    public VisionIO() {}

    public boolean hasTargets() { return false; }
    public double getNoteDistance() { return 0.0; }
    public double getNoteAngle() { return 0.0; }
    public double getLatency() { return 0.0; }

    public void updateInputs(VisionInputs inputs) {}
}