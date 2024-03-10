package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.VisionConstants;

public class VisionIOSim extends VisionIO {
    
    @Override
    public final boolean hasTarget() {
        return true;
    }
    
    // in meters
    @Override
    public final double getNoteDistance() {
        return 0.0;
    }
    
    // in radians
    @Override
    public final Pose2D getNoteAngle(Pose2D targetPose, double cameraToRobot) {
        return new Pose2d();
    }
    
    @Override
    public void updateInputs(IntakeInputs inputs) {
        
    }
}