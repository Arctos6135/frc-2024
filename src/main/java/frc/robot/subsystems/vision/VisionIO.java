package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.constants.VisionConstants;

public class VisionIO {
    private PhotonCamera photonCamera;

    @AutoLog
    public static class VisionInputs {
	
    }

    public void updateInputs(VisionInputs inputs) {}

    public final boolean hasTarget() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if(result.hasTargets()) {
            return true;
        }
        return false;
    }
    
    // in meters
    public final double getNoteDistance() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.cameraHeight,
                VisionConstants.targetHeigh,
                VisionConstants.cameraPitch,
                result.getBestTarget().getPitch() * .0175
            );
        }
        return 0.0;
    }
    
    // in radians
    public final Pose2D getNoteAngle(Pose2D targetPose, double cameraToRobot) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return PhotonUtils.estimateFieldToRobot(
                VisionConstants.cameraHeight,
                VisionConstants.targetHeigh,
                VisionConstants.cameraPitch,
                0,
                Rotation2d.fromDegrees(-result.getBestTarget.getYaw()),
                0, // gyro.getRotation2D() if gyro available
                targetPose,
                cameraToRobot
            );
        }
        return new Pose2D();
    }

    public VisionIO() {
        photonCamera = new PhotonCamera(VisionConstants.cameraName);
    }
}