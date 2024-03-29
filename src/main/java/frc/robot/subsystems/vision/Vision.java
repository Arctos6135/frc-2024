package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final VisionIO io;

    private final MedianFilter filter = new MedianFilter(40);
    private final MedianFilter dFilter = new MedianFilter(40);
    
    private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();
    
    @Override
    public void periodic() {
        Logger.processInputs("Vision", inputs);   
        io.updateInputs(inputs);
        Logger.recordOutput("Vision/Filtered Note Angle", filter.calculate(inputs.noteAngle));
        Logger.recordOutput("Vision/Filtered Note Distance", dFilter.calculate(inputs.noteDistance));
    }

    public Vision(VisionIO io) {
	    this.io = io;
    }

    public boolean hasTarget() {
        return inputs.hasTarget;
    }

    public Translation2d getTarget() {
        return new Translation2d(inputs.noteDistance, Rotation2d.fromRadians(inputs.noteAngle));
    }

    public double latency() {
        return inputs.latencyMillis;
    }
}