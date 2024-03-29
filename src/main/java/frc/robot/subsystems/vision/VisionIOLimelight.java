package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.VisionConstants;

public class VisionIOLimelight extends VisionIO {
    private PhotonCamera photonCamera;

    public VisionIOLimelight() {
        photonCamera = new PhotonCamera(VisionConstants.cameraName);
    }

    public boolean hasTarget() {
        return photonCamera.getLatestResult().hasTargets();
    }
    
    // in meters
    public double getNoteDistance() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.cameraHeight,
                VisionConstants.targetHeight,
                VisionConstants.cameraPitch,
                Units.degreesToRadians(result.getBestTarget().getPitch())
            );
        }
        return 0.0;
    }
    
    // in radians
    public double getNoteAngle() {//Pose2d targetPose, double cameraToRobot) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            // return PhotonUtils.estimateFieldToRobot(
            //     VisionConstants.cameraHeight,
            //     VisionConstants.targetHeight,
            //     VisionConstants.cameraPitch,
            //     0,
            //     Rotation2d.fromDegrees(-result.getBestTarget().getYaw()),
            //     0, // gyro.getRotation2D() if gyro available
            //     targetPose,
            //     cameraToRobot
            // );
            return Units.degreesToRadians(result.getBestTarget().getYaw());
        }
        return 0;
    }

    private double getNoteArea() {//Pose2d targetPose, double cameraToRobot) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getArea();
        }
        return 0;
    }

    public void updateInputs(VisionInputs inputs) {
        inputs.hasTarget = hasTarget();
        inputs.noteAngle = getNoteAngle();
        inputs.noteDistance = getNoteDistance();
        inputs.isConnected = photonCamera.isConnected();
        inputs.noteArea = getNoteArea();
    }

    // in seconds
    public double getLatency() {
        return photonCamera.getLatestResult().getLatencyMillis() / 1000;
    }
}